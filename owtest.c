#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "build/blink.pio.h"
#include "build/onewire.pio.h"
 
const uint LED_PIN = 25;
const uint DBG_PIN = 16;
 
void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);
    pio->txf[sm] = clock_get_hz(clk_sys) / 2 * freq;
}

struct owobj {
    PIO    pio;       /* pio object (pio0/pio1) */
    uint   sm;        /* state machine number */
    uint   offset;    /* onewire code offset in PIO instr. memory */
    uint   pin;       /* Pin number for 1wire data signal */
    uint   pinctlz;   /* Pin number for external FET strong pullup */

    // Search state global variables
    unsigned char ROM_NO[8];
    int LastDiscrepancy;
    int LastFamilyDiscrepancy;
    int LastDeviceFlag;
    unsigned char crc8;
};


/* Sample state machine instruction pointer for some time and
   print for diagnostic purposes */
void pio_sm_trace(PIO pio, uint sm, uint usleep)
{
    int  i;
    uint tracebuf[128];

    for(i=0; i<128; i++){
        tracebuf[i] = pio_sm_get_pc(pio, sm);
        sleep_us(usleep);
    }
    
    for(i=0; i<128; i++){
        printf("%d ", tracebuf[i]);
        if( (i+1)%16 == 0 ){
            printf("\n");
        }
    }
}

void onewire_set_fifo_thresh(struct owobj *owobj, uint thresh) {
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint old, new;
    uint8_t waiting_addr;
    uint need_restart;
    int timeout;
    
    if( thresh >= 32 ){
        thresh = 0;
    }

    old  = pio->sm[sm].shiftctrl;
    old &= PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS |
        PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS;

    new  = ((thresh & 0x1fu) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
        ((thresh & 0x1fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB);

    if( old != new ){
        need_restart = 0;
        if( pio->ctrl & (1u<<sm) ){
            /* If state machine is enabled, it must be disabled
               and restarted when we change fifo thresholds,
               or evil things happen */

            /* When we attempt fifo threshold switching, we assume
               that all fifo operations have been done and hence
               all bits have been almost processed, but the
               state machine might not have reached the wating state
               as it still does some delays to ensure timing for
               the very last bit (Similar for reset).
               Just wait for the 'wating' state to be reached */
            waiting_addr = offset+onewire_offset_waiting;
            timeout = 2000;
            while( pio_sm_get_pc(pio, sm) != waiting_addr ){
                sleep_us(1);
                if( timeout-- < 0 ){
                    /* FIXME: do something clever in case of
                       timeout */
                }
            }

            pio_sm_set_enabled(pio, sm, false);
            need_restart = 1;
        }

        hw_write_masked(&pio->sm[sm].shiftctrl, new,
                        PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS |
                        PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);

        if( need_restart ){
            pio_sm_restart(pio, sm);
            pio_sm_set_enabled(pio, sm, true);
        }
    }
}

int onewire_reset(struct owobj *owobj) {
    int ret;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint8_t waiting_addr;
    int timeout;
    uint div;
    
    /* Switch to slow timing for reset */
    div = clock_get_hz(clk_sys)/1000000 * 70;
    pio_sm_set_clkdiv_int_frac(pio, sm, div, 0);
    pio_sm_clkdiv_restart(pio, sm);
    onewire_set_fifo_thresh(owobj, 1);

    //onewire_do_reset(pio, sm, offset);
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + onewire_offset_reset));
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    ret = ((pio_sm_get(pio, sm) & 0x80000000) == 0 );
    
    /* when rx fifo has filled we still need to wait for
       the remaineder of the reset to execute before we
       can manipulate the clkdiv.
       Just wait until we reach the waiting state */
    waiting_addr = offset+onewire_offset_waiting;
    timeout = 2000;
    while( pio_sm_get_pc(pio, sm) != waiting_addr ){
        sleep_us(1);
        if( timeout-- < 0 ){
            /* FIXME: do something clever in case of
               timeout */
        }
    }

    /* Restore normal timing */
    div = clock_get_hz(clk_sys)/1000000 * 3;
    pio_sm_set_clkdiv_int_frac(pio, sm, div, 0);
    pio_sm_clkdiv_restart(pio, sm);

    return ret; // 1=detected, 0=not
}

/* Wait for idle state to be reached. This is only
   useful when you know that all but the last bit
   have been processe (after having checked fifos) */
void onewire_wait_for_idle(struct owobj *owobj) {
    int ret;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint8_t waiting_addr;
    int timeout;
    
    /* when rx fifo has filled the bit timing has not
       fully completed.
       Just wait until we reach the waiting state */
    waiting_addr = offset+onewire_offset_waiting;
    timeout = 2000;
    while( pio_sm_get_pc(pio, sm) != waiting_addr ){
        sleep_us(1);
        if( timeout-- < 0 ){
            /* FIXME: do something clever in case of
               timeout */
        }
    }

    return;
}

/* Transmit a byte */
void onewire_tx_byte(struct owobj *owobj, uint byte){
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;

    onewire_set_fifo_thresh(owobj, 8);
    pio->txf[sm] = byte;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    pio_sm_get(pio, sm); /* read to drain RX fifo */
    return;
}

/* Transmit a byte and activate strong pullup after
   last bit has been sent.
   Note: onewire_tx_byte_spu returns when the rx fifo
   has been read. This is 50 us prior to the end of the bit
   and hence 50 us prior to the strong pullup actually
   activated.
   Either consider this when controlling the strong
   pullup time or wait for idle before taking time. */
void onewire_tx_byte_spu(struct owobj *owobj, uint byte){
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;

    onewire_set_fifo_thresh(owobj, 7);
    pio->txf[sm] = byte;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    pio_sm_get(pio, sm); /* read to drain RX fifo */

    onewire_set_fifo_thresh(owobj, 1);
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0));
    pio->txf[sm] = byte>>7;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    pio_sm_get(pio, sm); /* read to drain RX fifo */

    return;
}

/* Reset the strong pullup (set pinctlz to high) */
void onewire_end_spu(struct owobj *owobj){
    PIO pio = owobj->pio;
    uint sm = owobj->sm;

    /* Preset y register so no SPU during next bit */
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 1));
    /* Set pinctlz pin to high ! */
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
}

/* Receive a byte */
uint onewire_rx_byte(struct owobj *owobj){
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;

    onewire_set_fifo_thresh(owobj, 8);
    pio->txf[sm] = 0xff;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    /* Returned byte is in 31..24 of RX fifo! */
    return (pio_sm_get(pio, sm) >> 24) & 0xff;
}

/* Do a ROM search triplet.
   Receive two bits and store the read values to
   id_bit and cmp_id_bit respectively.
   Then transmit a bit with this logic:
     id_bit | cmp_id_bit | tx-bit
          0 |          1 |      0
          1 |          0 |      1
          0 |          0 | search_direction     
          1 |          1 |      1
    The actually transmitted bit is returned via search_direction.
    
    Refer to MAXIM APPLICATION NOTE 187 "1-Wire Search Algorithm"
 */
void onewire_triplet(
    struct owobj *owobj,
    int *id_bit,
    int *cmp_id_bit,
    unsigned char *search_direction
)
{
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint fiforx;

    onewire_set_fifo_thresh(owobj, 2);
    pio->txf[sm] = 0x3;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;

    fiforx = pio_sm_get(pio, sm);
    *id_bit = (fiforx>>30) & 1;
    *cmp_id_bit = (fiforx>>31) & 1;
    if( (*id_bit == 0) && (*cmp_id_bit == 1) ){
        *search_direction = 0;
    } else if( (*id_bit == 1) && (*cmp_id_bit == 0) ){
        *search_direction = 1;
    } else if( (*id_bit == 0) && (*cmp_id_bit == 0) ){
        /* do not change search direction */
    } else {
        *search_direction = 1;
    }

    onewire_set_fifo_thresh(owobj, 1);
    pio->txf[sm] = *search_direction;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    fiforx = pio_sm_get(pio, sm);
    return;
}

static unsigned char calc_crc8_buf(unsigned char *data, int len)
{
    int            i,j;
    unsigned char  crc8;

    // See Application Note 27
    crc8 = 0;
    for(j=0; j<len; j++){
        crc8 = crc8 ^ data[j];
        for (i = 0; i < 8; ++i)
        {
            if (crc8 & 1)
                crc8 = (crc8 >> 1) ^ 0x8c;
            else
                crc8 = (crc8 >> 1);
        }
    }

    return crc8;
}

/* This is code stolen from MAXIM AN3684, slightly modified
   to interface to the PIO onewire and to eliminate global
   variables. */

#define TRUE    1
#define FALSE   0

typedef unsigned char byte;

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current 
// global 'crc8' value. 
// Returns current global crc8 value
//
unsigned char calc_crc8(struct owobj *owobj, unsigned char data)
{
    int i; 

    // See Application Note 27
    owobj->crc8 = owobj->crc8 ^ data;
    for (i = 0; i < 8; ++i)
    {
        if (owobj->crc8 & 1)
            owobj->crc8 = (owobj->crc8 >> 1) ^ 0x8c;
        else
            owobj->crc8 = (owobj->crc8 >> 1);
    }

    return owobj->crc8;
}

//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search.  This function
// continues from the previous search state. The search state
// can be reset by using the 'OWFirst' function.
//
// Returns:   TRUE (1) : when a 1-Wire device was found and its
//                       Serial Number placed in the global ROM 
//            FALSE (0): when no new device was found.  Either the
//                       last search was the last device or there
//                       are no devices on the 1-Wire Net.
//
int OWSearch(struct owobj *owobj)
{
    int id_bit_number;
    int last_zero, rom_byte_number, search_result;
    int id_bit, cmp_id_bit;
    unsigned char rom_byte_mask, search_direction, status;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = FALSE;
    owobj->crc8 = 0;

    // if the last call was not the last one
    if (!owobj->LastDeviceFlag)
    {       
        // 1-Wire reset
        if( ! onewire_reset(owobj) )
        {
            // reset the search
            owobj->LastDiscrepancy = 0;
            owobj->LastDeviceFlag = FALSE;
            owobj->LastFamilyDiscrepancy = 0;
            return FALSE;
        }

        // issue the search command
        onewire_tx_byte(owobj, 0xf0);

        // loop to do the search
        do
        {
            // if this discrepancy if before the Last Discrepancy
            // on a previous next then pick the same as last time
            if (id_bit_number < owobj->LastDiscrepancy)
            {
                if ((owobj->ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
                    search_direction = 1;
                else
                    search_direction = 0;
            }
            else
            {
                // if equal to last pick 1, if not then pick 0
                if (id_bit_number == owobj->LastDiscrepancy)
                    search_direction = 1;
                else
                    search_direction = 0;
            }

            // Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
            onewire_triplet(owobj, &id_bit, &cmp_id_bit, &search_direction);

            // check for no devices on 1-Wire
            if ((id_bit) && (cmp_id_bit))
                break;
            else
            {
                if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0))
                {
                    last_zero = id_bit_number;

                    // check for Last discrepancy in family
                    if (last_zero < 9)
                        owobj->LastFamilyDiscrepancy = last_zero;
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                    owobj->ROM_NO[rom_byte_number] |= rom_byte_mask;
                else
                    owobj->ROM_NO[rom_byte_number] &= (byte)~rom_byte_mask;

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    calc_crc8(owobj, owobj->ROM_NO[rom_byte_number]);  // accumulate the CRC
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!((id_bit_number < 65) || (owobj->crc8 != 0)))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            owobj->LastDiscrepancy = last_zero;

            // check for last device
            if (owobj->LastDiscrepancy == 0)
                owobj->LastDeviceFlag = TRUE;

            search_result = TRUE;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || (owobj->ROM_NO[0] == 0))
    {
        owobj->LastDiscrepancy = 0;
        owobj->LastDeviceFlag = FALSE;
        owobj->LastFamilyDiscrepancy = 0;
        search_result = FALSE;
    }

    return search_result;
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OWFirst(struct owobj *owobj)
{
    // reset the search state
    owobj->LastDiscrepancy = 0;
    owobj->LastDeviceFlag = FALSE;
    owobj->LastFamilyDiscrepancy = 0;

    return OWSearch(owobj);
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWNext(struct owobj *owobj)
{
    // leave the search state alone
    return OWSearch(owobj);
}

/* End of MAXIM AN3684 code */


/* Simple test with signle DS18B20
   Write and readback scratchpad register */
void onewire_test_ds18b20_scratchpad(struct owobj *owobj){
    int  i;
    unsigned char buf[9];
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint pinctlz =  owobj->pinctlz;
    
    onewire_program_init(pio, sm, offset, pin, pinctlz);
    pio_sm_set_enabled(pio, sm, true);

    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }

    onewire_tx_byte(owobj, 0xcc); // Skip ROM command
    onewire_tx_byte(owobj, 0x4e); // Write Scatchpad
    onewire_tx_byte(owobj, 0xa5); // TH
    onewire_tx_byte(owobj, 0x83); // TL
    onewire_tx_byte(owobj, 0x7f); // CONF

    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }

    onewire_tx_byte(owobj, 0xcc); // Skip ROM command
    onewire_tx_byte(owobj, 0xbe); // Read Scatchpad
    for(i=0; i<9; i++){
        buf[i] = onewire_rx_byte(owobj);
    }

    printf("Rx bytes:");
    for(i=0; i<9; i++){
        printf(" %.2x", buf[i]);
    }
    printf("\n");
    if( calc_crc8_buf(buf, 8) != buf[8] ){
        printf("CRC Fail\n");
        return;
    }
}

/* Simple test with single DS18B20
   Configure, start conversion and read temperature.
   Assume phantom power. */
void onewire_test_ds18b20_conversion(struct owobj *owobj){
    int  i;
    unsigned char buf[9];
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint pinctlz =  owobj->pinctlz;
    int32_t temp;
    
    onewire_program_init(pio, sm, offset, pin, pinctlz);
    pio_sm_set_enabled(pio, sm, true);

    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }

    onewire_tx_byte(owobj, 0xcc); // Skip ROM command
    onewire_tx_byte(owobj, 0x4e); // Write Scatchpad
    onewire_tx_byte(owobj, 0x00); // TH
    onewire_tx_byte(owobj, 0x00); // TL
    onewire_tx_byte(owobj, 0x7f); // CONF (12bit)

    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }

    onewire_tx_byte(owobj, 0xcc); // Skip ROM command
    onewire_tx_byte_spu(owobj, 0x44); // Convert T
    sleep_ms(800);  /* 12bit: 750 ms */
    onewire_end_spu(owobj);

    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }
    
    onewire_tx_byte(owobj, 0xcc); // Skip ROM command
    onewire_tx_byte(owobj, 0xbe); // Read Scatchpad
    for(i=0; i<9; i++){
        buf[i] = onewire_rx_byte(owobj);
    }
    
    if( calc_crc8_buf(buf, 8) != buf[8] ){
        printf("CRC Fail\n");
        return;
    }

    temp = buf[0]|(buf[1]<<8);
    if( temp & 0x8000 ){
        temp |= 0xffff0000;
    }
    printf("Temperature: %.3f\n", (float)temp/16);

    printf("Rx bytes:");
    for(i=0; i<9; i++){
        printf(" %.2x", buf[i]);
    }
    printf("\n");
}

void onewire_test_spu(struct owobj *owobj){
    int  i;
    unsigned char buf[9];
    uint  fiforx;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint pinctlz =  owobj->pinctlz;
    
    onewire_program_init(pio, sm, offset, pin, pinctlz);
    onewire_set_fifo_thresh(owobj, 2);
    pio_sm_set_enabled(pio, sm, true);

    onewire_tx_byte_spu(owobj, 0xff);
    onewire_wait_for_idle(owobj);
    sleep_us(150);
    onewire_end_spu(owobj);
    sleep_us(100);
    onewire_tx_byte_spu(owobj, 0x7f);
    sleep_us(350+50);
    onewire_end_spu(owobj);

}

/* Try testing with variable number of bits to be transmitted
   You should see 14 back-to-back high bits on the scope. */
void onewire_test_wordlength(struct owobj *owobj){
    int  i;
    unsigned char buf[9];
    uint  fiforx;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint pinctlz =  owobj->pinctlz;
    
    onewire_program_init(pio, sm, offset, pin, pinctlz);
    onewire_set_fifo_thresh(owobj, 2);
    pio_sm_set_enabled(pio, sm, true);

    //printf("idle\n");
    //pio_sm_trace(pio, sm);
    
    pio->txf[sm] = 0x3;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    fiforx = pio_sm_get(pio, sm);

    onewire_set_fifo_thresh(owobj, 3);
    pio->txf[sm] = 0x7;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    fiforx = pio_sm_get(pio, sm);

    onewire_set_fifo_thresh(owobj, 4);
    pio->txf[sm] = 0xf;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    fiforx = pio_sm_get(pio, sm);

    onewire_set_fifo_thresh(owobj, 5);
    pio->txf[sm] = 0x1f;
    while( pio_sm_get_rx_fifo_level(pio, sm) == 0 )
        /* wait */;
    fiforx = pio_sm_get(pio, sm);
}

/* ROM Search test */
void onewire_test_romsearch(struct owobj *owobj){
    int i;
    int ret;
    int id_bit, cmp_id_bit;
    unsigned char search_direction;
    PIO pio = owobj->pio;
    uint sm = owobj->sm;
    uint offset = owobj->offset;
    uint pin =  owobj->pin;
    uint pinctlz =  owobj->pinctlz;
    int devcount;
    
    onewire_program_init(pio, sm, offset, pin, pinctlz);
    pio_sm_set_enabled(pio, sm, true);

#if 0
    /* Simple test for onewire_triplet */
    if( ! onewire_reset(owobj) ){
        printf("No Device\n");
        return;
    }

    onewire_tx_byte(owobj, 0xf0); // ROM Search
    sleep_us(120);

    search_direction = 0;
    for(i=0; i<64; i++){
        onewire_triplet(owobj, &id_bit, &cmp_id_bit, &search_direction);
        printf("%d: %d\n", i, id_bit);
    }
#endif

#if 1
    /* Full flegged romsearch */
    printf("1Wire ROM search:\n");
    ret = OWFirst(owobj);
    devcount = 0;
    while( ret == TRUE ){
        devcount++;
        printf("%d:", devcount);
        for(i=0; i<8; i++){
            printf(" %.2x", owobj->ROM_NO[i]);
        }
        printf("\n");
        ret = OWNext(owobj);
    }
    if( devcount == 0 ){
        printf("No devices found\n");
    }
#endif
}


int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    PIO pio = pio0;
    uint offset_blink;
    uint offset_1wire;
    struct owobj owobj1;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(DBG_PIN);
    gpio_set_dir(DBG_PIN, GPIO_OUT);
    gpio_put(DBG_PIN, 0);

    stdio_init_all();
    puts("PIO: do onewire!\n");

    /* use sm 0-2 for blink test to learn if onwire
       code conflicts with this. */
    offset_blink = pio_add_program(pio, &blink_program);
    blink_pin_forever(pio, 0, offset_blink, 0, 3);
    blink_pin_forever(pio, 1, offset_blink, 6, 4);
    blink_pin_forever(pio, 2, offset_blink, 11, 1);

    /* Onewire config */
    offset_1wire = pio_add_program(pio, &onewire_program);
    owobj1.pio = pio;
    owobj1.sm = 3;
    owobj1.offset = offset_1wire;
    owobj1.pin = 15;
    owobj1.pinctlz = 14;

    while (1) {
    
        //onewire_test_ds18b20_scratchpad(&owobj1);
        //onewire_test_ds18b20_conversion(&owobj1);
        //onewire_test_wordlength(&owobj1);
        //onewire_test_spu(&owobj1);
        onewire_test_romsearch(&owobj1);

        gpio_put(LED_PIN, 0);
        sleep_ms(50);
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
    }
 }
