#include <mchck.h>
#include "labelprinter.desc.h"

enum {
        PIN_nRESET = PIN_PTD0,
        PIN_CLK = PIN_PTD1,
        PIN_DATA = PIN_PTD2,
        PIN_nLATCH = PIN_PTD3,
        PIN_nSTROBE = PIN_PTD4,
        PIN_MOTOR = PIN_PTD5,
        PIN_VUSB_SENSE = PIN_PTD6,
};

static const char *header_magic = "PRT";

struct print_header {
        char magic[sizeof(header_magic)];
        uint16_t lines;
};

enum input_state {
        IDLE,
        SPOOLING,
        PRINTING,
};


static void strobe_done_cb(enum pit_id id);
static void queue_line(void);


static struct cdc_ctx cdc;

static enum input_state input_state = IDLE;

static uint8_t input_buffer[512];
static size_t input_len = 0;
static int input_full = 0;

static uint16_t print_lines = 0;
static uint16_t printed_lines = 0;

static const int line_size = 8;
static const int buffer_lines = 40;

static const int strobe_on_us = 3540;   /* 4ms */
static const int strobe_off_us = 10400; /* 10ms */


static uint32_t
us_to_cycles(uint32_t us)
{
        return (us * 48);
}

static void
enable_pins(void)
{
        pin_mode(PIN_nRESET, PIN_MODE_OPEN_DRAIN_ON);
        gpio_write(PIN_nRESET, GPIO_LOW);
        gpio_dir(PIN_nRESET, GPIO_OUTPUT);

        /* controller is in reset now */

        pin_mode(PIN_CLK, PIN_MODE_MUX_ALT2);
        pin_mode(PIN_DATA, PIN_MODE_MUX_ALT2);

        gpio_write(PIN_nLATCH, GPIO_HIGH);
        gpio_dir(PIN_nLATCH, GPIO_OUTPUT);

        gpio_write(PIN_nSTROBE, GPIO_HIGH);
        gpio_dir(PIN_nSTROBE, GPIO_OUTPUT);

        gpio_write(PIN_MOTOR, GPIO_LOW);
        gpio_dir(PIN_MOTOR, GPIO_OUTPUT);
}

static void
disable_pins(void)
{
        pin_mode(PIN_CLK, PIN_MODE_MUX_ANALOG);
        pin_mode(PIN_DATA, PIN_MODE_MUX_ANALOG);
        gpio_write(PIN_MOTOR, GPIO_LOW);
        gpio_dir(PIN_MOTOR, GPIO_DISABLE);
        gpio_write(PIN_nSTROBE, GPIO_HIGH);
        gpio_dir(PIN_nSTROBE, GPIO_DISABLE);
        gpio_write(PIN_nLATCH, GPIO_HIGH);
        gpio_dir(PIN_nLATCH, GPIO_DISABLE);
        gpio_dir(PIN_nRESET, GPIO_DISABLE);
        onboard_led(ONBOARD_LED_FLOAT);
}

static void
setup_print(uint16_t lines)
{
        printed_lines = 0;
        print_lines = lines;
        enable_pins();
}

static void
cycle_done_cb(enum pit_id id)
{
        if (printed_lines == print_lines) {
                pit_stop(PIT_0);
                disable_pins();
                input_state = IDLE;
        } else {
                pit_start(PIT_1, us_to_cycles(strobe_on_us), strobe_done_cb);
                gpio_write(PIN_nSTROBE, GPIO_LOW);
        }

        if (printed_lines % 72 == 0 || printed_lines == print_lines)
                printf("P%d\n", printed_lines);
}

static void
latch_data(void)
{
        gpio_write(PIN_nLATCH, GPIO_LOW);
        for (volatile int i = 40; i > 0; --i)
                /* NOTHING */;
        gpio_write(PIN_nLATCH, GPIO_HIGH);
}

static void
strobe_done_cb(enum pit_id id)
{
        pit_stop(PIT_1);
        gpio_write(PIN_nSTROBE, GPIO_HIGH);

        latch_data();
        queue_line();
}

static void
read_more(void)
{
        if (sizeof(input_buffer) - input_len >= CDC_RX_SIZE) {
                input_full = 0;
                cdc_read_more(&cdc);
        } else {
                input_full = 1;
        }
}

static void
line_sent_cb(void *cbdata)
{
        crit_enter();
        input_len -= line_size;
        memcpy(input_buffer, input_buffer + line_size, input_len);
        if (input_full)
                read_more();
        crit_exit();

        /* first line? -> switch on motor */
        if (printed_lines == 0) {
                latch_data();
                pit_start(PIT_0, us_to_cycles(strobe_on_us + strobe_off_us), cycle_done_cb);
                pit_start(PIT_1, us_to_cycles(strobe_on_us), strobe_done_cb);
                gpio_write(PIN_nSTROBE, GPIO_LOW);
                gpio_write(PIN_MOTOR, GPIO_HIGH);
        }
        printed_lines++;
}

static void
queue_line(void)
{
        static struct spi_ctx spi;

        if (input_state != PRINTING)
                return;

        if (input_len < line_size) {
                /* buffer underrun */
                pit_stop(PIT_0);
                disable_pins();
                printf("U%d\n", printed_lines);
                input_state = IDLE;
                return;
        }

        spi_queue_xfer(&spi, 0,
                       input_buffer, line_size,
                       NULL, 0,
                       line_sent_cb, NULL);
}

static void
start_print(void)
{
        /* buffer some more? */
        if (input_len < buffer_lines * line_size &&
            input_len < print_lines * line_size)
                return;

        onboard_led(ONBOARD_LED_ON);
        input_state = PRINTING;
        queue_line();
}

static void
serial_in(uint8_t *data, size_t len)
{
        size_t copylen = len;

        if (sizeof(input_buffer) - input_len < copylen)
                copylen = sizeof(input_buffer) - input_len;

        memcpy(input_buffer + input_len, data, copylen);
        input_len += copylen;

        uint8_t *c = input_buffer;
        switch (input_state) {
        case IDLE:
                for (; c < input_buffer + input_len - sizeof(struct print_header); ++c) {
                        struct print_header *h = (void *)c;

                        if (memcmp(h->magic, header_magic, sizeof(h->magic)) == 0) {
                                setup_print(h->lines);
                                input_state = SPOOLING;
                                c += sizeof(*h);
                                break;
                        }
                }
                /* FALLTHROUGH */


        case SPOOLING:
        case PRINTING:
                if (c > input_buffer) {
                        memcpy(input_buffer, c, input_len - (c - input_buffer));
                        input_len -= c - input_buffer;
                }

                read_more();

                if (input_state == SPOOLING)
                        start_print();
                break;
        }
}


void
init_serial(int config)
{
        cdc_init(serial_in, NULL, &cdc);
        cdc_set_stdout(&cdc);
        dfu_app_init(sys_reset_to_loader);
}

void
suspend_sys(void)
{
        /* XXX check if print job is running */
        disable_pins();

        SMC.pmctrl.stopm = STOPM_VLPS;
        SCB.scr.sleepdeep = 1;
}

void
resume_sys(void)
{
        SCB.scr.sleepdeep = 0;
}


/* No voltage on VUSB -> go into VLLS0 */
static void
power_down_cb(void *cbdata)
{
        crit_enter();

        disable_pins();

        /* PTD6 is P15, which is wupe3 in wupe[3], and bit 7 in wuf2 */

        /* clear old condition */
        LLWU.wuf2 |= 1 << 7;
        /* set up wakeup condition */
        LLWU.wupe[3].wupe3 = LLWU_PE_RISING;

        /* set up VLLS0 */
        SMC.pmctrl.stopm = STOPM_VLLS;
        SMC.vllsctrl.porpo = 0;
        SMC.vllsctrl.vllsm = VLLSM_VLLS0;

        /* did it come back? */
        if (gpio_read(PIN_VUSB_SENSE))
                goto error;

        /* set up deep sleep in wfi */
        SCB.scr.sleepdeep = 1;

        /* either we come back with stopa set, or we return via reset */
        asm("wfi");

error:
        /* wut? something went wrong.  abort. */
        SCB.scr.sleepdeep = 0;
        sys_reset();
}
PIN_DEFINE_CALLBACK(PIN_PTD6, PIN_CHANGE_ZERO, power_down_cb, NULL);

int
main(void)
{
        /* we want to go into VLLS and VLPS modes */
        SMC.pmprot.raw = ((struct SMC_PMPROT) { .avlls = 1, .avlp = 1 }).raw;

        /* Disable the JTAG/SWD pins */
        pin_mode(PIN_PTA0, PIN_MODE_MUX_ANALOG);
        pin_mode(PIN_PTA1, PIN_MODE_MUX_ANALOG);
        pin_mode(PIN_PTA2, PIN_MODE_MUX_ANALOG);
        pin_mode(PIN_PTA3, PIN_MODE_MUX_ANALOG);
        pin_mode(PIN_PTA4, PIN_MODE_MUX_ANALOG);
        SIM.scgc5.porta = 0;

        /* disable USB regulator */
        SIM.sopt1.usbregen = 0;

        /* return from VLLS0 */
        gpio_dir(PIN_VUSB_SENSE, GPIO_INPUT);
        PMC.regsc.ackiso = 1;

        pin_change_init();
        spi_init();
        pit_init();
        usb_init(&cdc_device);

        sys_yield_for_frogs();
}
