#include <inttypes.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "stk500v2.h"

// NOTE: naked does not allocate a stack frame -- use OS_main
extern "C" void __init(void) __attribute__ ((OS_main,externally_visible));

#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))

static inline void watchdog(uint8_t x) { // Call only with interrupts disabled
    asm volatile ("wdr");
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = x;
}

#define EP_CONTROL_OUT 0x00

#define EP_ISOCHRONOUS 0x40
#define EP_BULK 0x80
#define EP_INTERRUPT 0xC0
#define EP_OUT 0x00
#define EP_IN 0x01

#define EP_SIZE_8 0x00
#define EP_SIZE_16 0x10
#define EP_SIZE_32 0x20
#define EP_SIZE_64 0x30
#define EP_SIZE_128 0x40
#define EP_SIZE_256 0x50
#define EP_SINGLE 0x02
#define EP_DOUBLE 0x06
#define EP_FREE 0x00

#define DEVICE_TO_HOST        0x80
#define REQUEST_TYPE_MASK     0x60
#define REQUEST_TYPE_STANDARD 0x00
#define REQUEST_TYPE_CLASS    0x20
#define REQUEST_TYPE_VENDOR   0x40
#define RECIPIENT_MASK        0x1F
#define RECIPIENT_DEVICE      0x00
#define RECIPIENT_INTERFACE   0x01
#define RECIPIENT_ENDPOINT    0x02
#define RECIPIENT_OTHER       0x03

#define GET_STATUS            0
#define CLEAR_FEATURE         1
#define SET_FEATURE           3
#define SET_ADDRESS           5
#define GET_DESCRIPTOR        6
#define SET_DESCRIPTOR        7
#define GET_CONFIGURATION     8
#define SET_CONFIGURATION     9
#define GET_INTERFACE         10
#define SET_INTERFACE         17
#define SYNCH_FRAME           18

#define GET_REPORT            1
#define SET_REPORT            9

#define __cli() asm volatile ("cli")
#define __sei() asm volatile ("sei")
#define __sleep() asm volatile ("sleep")

#ifdef RAMPZ
#define LPM "elpm"
#else
#define LPM "lpm"
#endif

extern inline uint8_t pgm_read_byte_boot(PGM_P p) {
    uint8_t result;
    asm (
        LPM " %0, Z"
        : "=r" (result)
        : "z" (p)
    );
    return result;
}

#define pgm_read_byte_inc(p) ({\
    uint8_t result; \
    asm ( \
        LPM " %0, Z+" \
        : "=r" (result) \
        , "=z" (p) \
        : "1" (p) \
    ); \
    result; \
})

#define be16_load(r, p) asm ( \
    "lds %B0, %1\n\t" \
    "lds %A0, %1+1\n\t" \
    : "=r" (r) \
    : "p" (p) \
)

#define byte_sat(n) ({ \
    uint8_t result; \
    asm ( \
        "tst %B1\n\t" \
        "brne 1f\n\t" \
        "mov %0, %A1\n" \
        "1:\n\t" \
        : "=r" (result) \
        : "r" (n) \
        , "0" (0xff) \
    ); \
    result; \
})

// Routines stolen from boot.h because it uses eeprom.h which won't compile under -mint8.

#define boot_signature_byte_get(addr) ({ \
    uint8_t result; \
    asm ( \
        "out %1-32, %2\n\t" \
        "lpm %0, Z\n\t" \
        : "=r" (result) \
        : "i" (_SFR_MEM_ADDR(SPMCSR)) \
        , "r" (_BV(SPMEN) | _BV(SIGRD)) \
        , "z" (addr) \
    ); \
    result; \
})

#define boot_page_erase(addr) asm volatile ( \
    "out %0-32, %1\n\t" \
    "spm\n\t" \
    : \
    : "i" (_SFR_MEM_ADDR(SPMCSR)) \
    , "r" (_BV(SPMEN) | _BV(PGERS)) \
    , "z" (addr) \
)

#define boot_page_write(addr) asm volatile ( \
    "out %0-32, %1\n\t" \
    "spm\n\t" \
    : \
    : "i" (_SFR_MEM_ADDR(SPMCSR)) \
    , "r" (_BV(SPMEN) | _BV(PGWRT)) \
    , "z" (addr) \
)

#define boot_rww_enable() asm volatile ( \
    "out %0-32, %1\n\t" \
    "spm\n\t" \
    : \
    : "i" (_SFR_MEM_ADDR(SPMCSR)) \
    , "r" (_BV(SPMEN) | _BV(RWWSRE)) \
)

#define boot_spm_busy() (SPMCSR & _BV(SPMEN))

#define boot_spm_busy_wait() do {} while (boot_spm_busy())

extern "C" const prog_char device_descriptor[];
extern "C" const prog_char configuration_descriptor[];
extern "C" const prog_char string_descriptors[];

static bool control_read(uint8_t *d, uint8_t l) {
    // CORNER: l == 0 must still read a packet?
    do {
        uint8_t ueintx = UEINTX;
#ifdef IMPORTANT_CORNERS
        // Without this check, we can get stuck in this loop.
        if (ueintx & _BV(RXSTPI)) {
           // Host aborted the transaction.
           return false;
        }
#endif
#ifdef CORNERS
        if (ueintx & _BV(RXOUTI)) {
            // IN packet: host stopped sending data.
            return false;
        }
#endif
        if (ueintx & _BV(RXOUTI)) {
            while (l && UEBCLX) {
                *d++ = UEDATX;
                --l;
            }
            UEINTX = ~_BV(RXOUTI);
        }
    } while (l);
    return true;
}

static bool control_write(const uint8_t *d, uint8_t l, bool progmem) {
    // Number of bytes left in the IN packet buffer.
    uint8_t bytes_sent = 0x40;
    do {
        uint8_t ueintx = UEINTX;
#ifdef IMPORTANT_CORNERS
        // Without this check, we can get stuck in this loop.
        if (ueintx & _BV(RXSTPI)) {
           // SETUP packet(?): Host aborted the transaction.
           return false;
        }
#endif
#ifdef CORNERS
        if (ueintx & _BV(RXOUTI)) {
            // OUT packet: host stopped asking for data.
            break;
        }
#endif
        if (ueintx & _BV(TXINI)) {
            // New IN packet.
            bytes_sent = UEBCLX;
            // Push bytes until buffer full or no more bytes.
            while (l && bytes_sent < 0x40) {
                UEDATX = progmem ? pgm_read_byte_inc(d) : *d++;
                --l;
                ++bytes_sent;
            }
            UEINTX = ~_BV(TXINI);
        }
    // We expect another IN packet if the last packet was full.
    } while (bytes_sent == 0x40);
    return true;
}

#define WORD(x) (x & 0xff), (x >> 8)

static const prog_char report_descriptor[] PROGMEM = {
    0x06, WORD(0xff9c), // Usage page (Vendor-specific)
    0x09, 0x1b,         // Usage (0x1b)
    0xa1, 0x01,         // Collection (Application)
    0x75, 8,            // Report size (8)
    0x15, -128,         // Logical minimum (-128)
    0x25, 127,          // Logical maximum (127)

#define FEATURE(id, n) 0x85, id, 0x95, n, 0x08, 0xb2, WORD(0x0102)
    // Report ID (id)
    // Report count (n)
    // Usage (Undefined)
    // Feature (buffered bytes, variable)
    FEATURE(1, 14),
    FEATURE(2, 30),
    FEATURE(3, 62),
    FEATURE(4, 126),
    0xc0,               // End collection
};

static bool send_descriptor(uint8_t type, uint8_t index, uint8_t length) {
    // Hack to pass the report descriptor length to the configuration descriptor.
    __asm__ (".global report_descriptor_len\n.set report_descriptor_len, %0" : : "I" (sizeof(report_descriptor)));
    PGM_P d;
    uint8_t l;
    switch (type) {
    case 1:
        d = device_descriptor;
        l = pgm_read_byte_boot(d);
        break;
    case 2:
        d = configuration_descriptor;
        // XXX: This is technically a word.
        l = pgm_read_byte_boot(d + 2);
        break;
    case 3:
        d = string_descriptors + index;  // FIXME: Needs range check.
        l = pgm_read_byte_boot(d);
        break;
    case 0x22:
        d = report_descriptor;
        l = sizeof(report_descriptor);
        break;
    default:
        return false;
    }
    if (l > length) l = length;
    return control_write((const uint8_t *) d, l, true);
}

static uint8_t sendbuf[512];

register uint16_t sendstart asm("r16");
register uint16_t sendlen asm("r14");
register uint16_t cur_addr asm("r12");
register uint16_t flash_remain asm("r10");
register uint8_t flash_low_byte asm("r9");
#ifdef RAMPZ
register uint8_t rampz asm("r8");
#endif

static void bufinit(void) {
  sendlen = 0;
  flash_remain = 0;
}

static bool send(uint8_t report_id) {
    UEDATX = report_id;
    UEDATX = byte_sat(sendlen);

    // Report size = (8 << report id) - 2
    // n = report size + 1 (for the report id)
    uint8_t n = 8;
#if 0
    n <<= report_id;
#else
    asm (
        "\n1:\n\t"
        "lsl %0\n\t"
        "subi %1, 1\n\t"
        "brne 1b\n\t"
        : "=r" (n)
        : "r" (report_id)
        , "0" (n)
    );
#endif
    --n;

    // l = reamining bytes in report
    uint8_t l = n - 2;
    uint8_t *buf = sendbuf + sendstart;
    uint16_t s = l;
    if (s > sendlen)
        s = sendlen;
    sendstart += s;
    sendlen -= s;
    return control_write(buf, l, false);
}

static const prog_char sign_on_response[] PROGMEM = {
    8,
    'S', 'T', 'K', '5', '0', '0', '_', '2',
};

static bool recv(uint8_t length) {
    uint16_t pktlen = 2;
    uint8_t status = STATUS_CMD_OK;

    uint8_t *readptr = sendbuf;
    if (flash_remain) {
        readptr = sendbuf + 15;
    }

    if (!control_read(readptr, length))
        return false;

    if (flash_remain) {
        length += 15;
        goto resume_flash;
    }

    // Command overrun or final byte of a flash write.
    if (sendlen)
        return true;

    // First two bytes are report ID and length
    sendstart = 2;
    switch (sendbuf[7]) {
    case CMD_SIGN_ON: {
        register PGM_P inptr asm("r30") = sign_on_response;
        for (uint8_t i = 0; i < sizeof(sign_on_response); i++) {
            sendbuf[9+i] = pgm_read_byte_inc(inptr);
        }
        pktlen = 3 + sizeof(sign_on_response);
        break;
    }
    case CMD_ENTER_PROGMODE_ISP:
    case CMD_LEAVE_PROGMODE_ISP:
        break;
    case CMD_SPI_MULTI:
        // decode sendbuf[11..14] and respond with sendbuf[9..12]
        if (sendbuf[11] == 0x30) {
            // read signature
            sendbuf[12] = boot_signature_byte_get(sendbuf[13] << 1);
        }
        pktlen = 6;
        break;
    case CMD_LOAD_ADDRESS:
#ifdef CORNERS
        if (sendbuf[8] || sendbuf[9]) {
            status = STATUS_CMD_FAILED;
            break;
        }
#endif
        be16_load(cur_addr, sendbuf + 10);
#ifdef RAMPZ
        rampz = cur_addr >> 15;
#endif
        cur_addr <<= 1;
        break;
    case CMD_READ_FLASH_ISP: {
#ifdef CORNERS
        uint16_t tmp;
        be16_load(tmp, sendbuf + 8);
        if (tmp == 0 || tmp > 0x100) {
            status = STATUS_CMD_FAILED;
            break;
        }
#endif
#ifdef RAMPZ
        RAMPZ = rampz;
#endif
        uint8_t n = sendbuf[9] - 1;
        pktlen = (uint16_t) n + 4;
        uint8_t *buf = sendbuf + 9;
        asm volatile (
            "\n1:\n\t"
            LPM " __tmp_reg__, %a0+\n\t"
            "st %a1+, __tmp_reg__\n\t"
            "subi %2, 1\n\t"
            "brsh 1b\n\t"
            : "=z" (cur_addr)
            , "=e" (buf)
            : "r" (n)
            , "0" (cur_addr)
            , "1" (buf)
        );
        *buf = STATUS_CMD_OK;
        break;
    }
    case CMD_CHIP_ERASE_ISP:
        // You want a chip erase? That's nice.
        break;
    case CMD_PROGRAM_FLASH_ISP: {
        be16_load(flash_remain, sendbuf + 8);
resume_flash:
#ifdef RAMPZ
        RAMPZ = rampz;
#endif
        uint16_t page_base = cur_addr;
        uint8_t *p = sendbuf + 17;
        uint8_t *pmax = sendbuf + length;
        do {
            if (flash_remain & 1) {
                asm volatile (
                    "mov r0, %1\n\t"
                    "ld r1, %a0+\n\t"
                    "out %2-32, %3\n\t"
                    "spm\n\t"
                    "clr r1\n\t"
                    : "=e" (p)
                    : "r" (flash_low_byte)
                    , "i" (_SFR_MEM_ADDR(SPMCSR))
                    , "r" (_BV(SPMEN))
                    , "z" (cur_addr)
                    , "0" (p)
                    : "r0"
                );
                cur_addr += 2;
		boot_spm_busy_wait();
            } else {
                flash_low_byte = *p++;
            }
            --flash_remain;
        } while (flash_remain && p < pmax);
        if (flash_remain) {
            // Ready to resume.
            goto out;
        }
        if (sendbuf[10] & 128) {
            boot_page_erase(page_base);
            boot_spm_busy_wait();
            boot_page_write(page_base);
            boot_spm_busy_wait();
            boot_rww_enable();  // XXX: possible corner
        }
        break;
    }
    default:
        status = STATUS_CMD_UNKNOWN;
        break;
    }
    sendbuf[4] = pktlen >> 8;
    sendbuf[5] = pktlen & 255;
    sendlen = pktlen + 6;
    sendbuf[8] = status;

    {  // Hide this stuff from "goto out" above.
        uint8_t *ptr = sendbuf + sendstart;
        uint8_t *ptrend = ptr + sendlen - 1;
        uint8_t c = 0;
        while (ptr < ptrend) {
            c ^= *ptr++;
        }
        *ptr = c;
    }

out:
#ifdef RAMPZ
    RAMPZ = 1;
#endif
    return true;
}

static void handle_setup(void) {
    union {
        uint8_t raw[8];
        struct {
            uint8_t request_type;
            uint8_t request;
            uint16_t value;
            uint16_t index;
            uint16_t length;
        } pkt;
    } setup;
    for (uint8_t i = 0; i < 8; ++i) {
        setup.raw[i] = UEDATX;
    }
    UEINTX = ~_BV(RXSTPI);

    switch (setup.pkt.request_type & ~DEVICE_TO_HOST) {
    case REQUEST_TYPE_STANDARD | RECIPIENT_DEVICE:
    case REQUEST_TYPE_STANDARD | RECIPIENT_INTERFACE:  // cheating!
        switch (setup.pkt.request) {
#ifdef CORNERS
        case GET_STATUS:
            UEDATX = 0;  // should be 1 for device requests (self-powered)
            UEDATX = 0;
            break;
#endif
        case SET_ADDRESS:
            // Clear status stage
            UEINTX = ~_BV(TXINI);
            while (!(UEINTX & _BV(TXINI)));
#ifdef CORNERS
            UDADDR = setup.pkt.value & 0xff;
#endif
            UDADDR = setup.pkt.value & 0xff | _BV(ADDEN);
            return;
#ifdef CORNERS
        case GET_CONFIGURATION:
            UEDATX = 1;
            break;
#endif
        case SET_CONFIGURATION:
            bufinit();
            break;
        case GET_DESCRIPTOR:
            if (send_descriptor(setup.pkt.value >> 8, setup.pkt.value & 0xff, byte_sat(setup.pkt.length)))
                break;
            goto badrequest;
        default:
            goto badrequest;
        }
        break;
    case REQUEST_TYPE_CLASS | RECIPIENT_INTERFACE:
        switch (setup.pkt.request) {
        case GET_REPORT:
            if (send(setup.pkt.value & 0xff))
                break;
            goto badrequest;
        case SET_REPORT:
            //if (recv(byte_sat(setup.pkt.length)))
            if (recv(setup.pkt.length))
                break;
            goto badrequest;
        default:
            // Yes, we get other requests. :(
            goto badrequest;
        }
        break;
    default:
        goto badrequest;
    }

    UEINTX = ~_BV(TXINI);  // done sending data *or* clear IN status stage
#ifdef CORNERS
    /* No clue if this code is correct. */
    if (setup.pkt.request_type & DEVICE_TO_HOST) {
        while (!(UEINTX & _BV(TXINI)));
        UEINTX = ~_BV(RXOUTI);  // clear status stage
    }
#endif
    return;

badrequest:
    UECONX = _BV(STALLRQ) | _BV(EPEN);  // stall = error
}

// Declaration above.
void __init(void) {
    asm volatile ("clr __zero_reg__");
#ifdef RAMPZ
    RAMPZ = 1;
#endif

    // TODO: Check MCUSR in case we're crashlooping.
    //MCUSR = 0;
    //watchdog(WATCHDOG_OFF);

    // USB setup.
    UHWCON = _BV(UVREGE);  // Turn on pad regulator.
    // Must freeze clocks or you don't get an EORST.
    USBCON = _BV(USBE) | _BV(FRZCLK);
    // Start up the PLL now.
    #if defined(__AVR_AT90USB646__)
    PLLCSR = 0x1A;
    #elif defined(__AVR_AT90USB1286__)
    PLLCSR = 0x16;
    #else
    #error Not a 646 or 1286 - check your configuration.
    #endif

    // Wait for USB PLL to lock.
    while (!(PLLCSR & _BV(PLOCK)));

    // Configure LED pin as output.
    DDRD = _BV(PD6);
#ifdef FLASH_LED
    // CTC, f div 1024
    TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
    OCR1A = 16000000LL / 1024 / 2;  // 1Hz
    TCNT1 = 0;
    TIFR1 = _BV(OCF1A);
#endif
#ifndef CHEAP_LED
    uint16_t led_counter = 0;
#endif

    // Enable clocks and OTG pin.
    USBCON = _BV(USBE) | _BV(OTGPADE);
    UDCON = 0;  // Attach at full speed.
    UENUM = 0;  // Access endpoint 0.

    for (;;) {
        if (UDINT & _BV(EORSTI)) {
            UDINT = 0;

            // Initialize endpoint 0.
            UECONX = _BV(EPEN);  // Enable endpoint.
#ifdef CORNERS
            UECFG1X = 0;
#endif
            UECFG0X = EP_CONTROL_OUT;
            UECFG1X = EP_SIZE_64 | EP_SINGLE;
        }
        if (UEINTX & _BV(RXSTPI)) {
            handle_setup();
        }
#ifdef FLASH_LED
        if (TIFR1 & _BV(OCF1A)) {
            TIFR1 = _BV(OCF1A);  // XXX what does this do?
            PORTD = ~PORTD;
        }
#endif
#ifndef CHEAP_LED
        asm volatile (
            "sec\n\t"
            "sez\n\t"
            "sbc %A0, __zero_reg__\n\t"
            "sbc %B0, __zero_reg__\n\t"
            "brne 1f\n\t"
            "in r0, %1-32\n\t"
            "com r0\n\t"
            "out %1-32, r0\n\t"
            "1:\n\t"
            : "=r" (led_counter)
            : "I" (_SFR_MEM_ADDR(PORTD))
            , "0" (led_counter)
        );
#endif
    }
}

// vim:set sw=4:
