    .section .vectors
    rjmp __init

    .section .progmem
    .global device_descriptor
device_descriptor:
0:
    .byte 0f - 0b
    .byte 1             ; device descriptor
    .word 0x200         ; USB v2.0
    .byte 2             ; CDC
    .byte 0             ; No defined subclasses
    .byte 0             ; No defined protocols
    .byte 64            ; 64-byte FIFO on control port
    .word 0x16c0        ; Vendor ID
    .word 0x05df        ; Product ID (HID shared obdev.at)
    .word 0x100         ; Device v1.0
    .byte manufacturer_string - string_descriptors
    .byte product_string - string_descriptors
    .byte 0             ; No serial number string
    .byte 1             ; 1 configuration

    .global configuration_descriptor
configuration_descriptor:
1:
0:
    .byte 0f - 0b
    .byte 2             ; configuration descriptor
    ;.word 1f - 1b
    .byte 1f - 1b, 0
    .byte 1             ; 1 interface
    .byte 1             ; SET_CONFIGURATION(1)
    .byte 0             ; No string description
    .byte 0xC0          ; Self powered, no remote wakeup
    .byte 10/2          ; 10mA power draw
0:
    .byte 0f - 0b
    .byte 4             ; interface descriptor
    .byte 0             ; interface number
    .byte 0             ; no alternate settings
    .byte 1             ; 1 endpoint in the interface
    .byte 3             ; Human Interface Class
    .byte 0             ; No subclass
    .byte 0             ; No protocol
    .byte 0             ; No string description
0:
    .byte 0f - 0b
    .byte 0x21          ; HID descriptor
    .word 0x111         ; USB HID Specification Release 1.11
    .byte 0             ; No country code: not localized
    .byte 1             ; 1 descriptor
    .byte 0x22          ; HID report descriptor type
    .word report_descriptor_len
0:
    .byte 0f - 0b
    .byte 5             ; endpoint descriptor
    .byte 0x81          ; endpoint 1, IN
    .byte 0b11          ; interrupt type
    .word 8             ; max packet size
    .byte 255           ; polling interval
0:
1:
0:
    .global string_descriptors
string_descriptors:
0:
    .byte 0f - 0b
    .byte 3             ; string descriptor
    .word 0x0409        ; US English language code
manufacturer_string:
0:
    .byte 0f - 0b
    .byte 3             ; string descriptor
    .word 'o', 'b', 'd', 'e', 'v', '.', 'a', 't'
product_string:
0:
    .byte 0f - 0b
    .byte 3             ; string descriptor
    .word 'A', 'V', 'R', '-', 'D', 'o', 'p', 'e', 'r'
0:
