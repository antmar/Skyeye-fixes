/*
	skyeye_mach_ep9312.c - define machine ep9312 for skyeye
	Copyright (C) 2003 Skyeye Develop Group
	for help please send mail to <skyeye-developer@lists.sf.linuxforum.net> 
	
	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 
*/
/*
 * 11/06/2004	clean some codes
 *		wlm <wlm@student.dlut.edu.cn>
 * 10/8/2004 	init this file.
 * 		add machine ep9312's function. 
 *		Cai Qiang <caiqiang@ustc.edu> 		
 *
 * */

#include "armdefs.h"
#include "clps9312.h"
#include "ep9312.h"
#include "serial_amba.h"
//zzc:2005-1-1
#ifdef __CYGWIN__
//chy 2005-07-28
#include <time.h>
//teawater add DBCT_TEST_SPEED 2005.10.04---------------------------------------
/*struct timeval
{
	int tv_sec;
	int tv_usec;
};*/
//AJ2D--------------------------------------------------------------------------
#endif

/* 2007-01-18 added by Anthony Lee : for new uart device frame */
#include "skyeye_uart.h"


void ep9312_io_write_word (ARMul_State * state, ARMword addr, ARMword data);
ARMword ep9312_io_read_word (ARMul_State * state, ARMword addr);



#define NR_UART			3
#define NR_VIC			2
#define NR_TC			3

#define UART_FR_TXFE	(1<<7)
#define UART_FR_RXFE	(1<<4)

#define UART_IIR_RIS	(1<<1)
#define UART_IIR_TIS	(1<<2)

typedef struct ep9312_tcoi {
    int vic;
    int bit;
} ep9312_tcoi_t;

const ep9312_tcoi_t TCOI[3] = { {0, 1 << 4}, {0, 1 << 5}, {1, 1 << (51 - 32)} };
const int UART_RXINTR[3] = { 1 << 23, 1 << 25, 1 << 27 };
const int UART_TXINTR[3] = { 1 << 24, 1 << 26, 1 << 28 };
const int INT_UART[3] = { 1 << (52 - 32), 1 << (54 - 32), 1 << (55 - 32) };
const int iConsole = 0;		//index of uart of serial console

/* Helper functions for working with the VICs */
static void VIC_init( int vic );
static u32 VIC_irq_status( int vic );
static u32 VIC_fiq_status( int vic );
static u32 VIC_int_select( int vic );
static void VIC_select_int( int vic, u32 bits );
static u32 VIC_int_enabled( int vic );
static void VIC_enable_int( int vic, u32 bits );
static void VIC_disable_int( int vic, u32 bits );
static u32 VIC_raw_int( int vic );
static void VIC_raise_int( int vic, u32 bits );
static void VIC_clear_int( int vic, u32 bits );
static u32 VIC_int_soft_status( int vic );
static void VIC_raise_soft_int( int vic, u32 bits );
static void VIC_clear_soft_int( int vic, u32 bits );
/*
 * Required for vectored interrupts
static u32 VIC_addr( int vic );
static u32 VIC_def_addr( int vic );
static void VIC_set_def_addr( int vic, u32 bits );
static u32 VIC_vec_addr( int vic, int slot );
static void VIC_set_vec_addr( int vic, int slot, u32 addr );
static u32 VIC_vec_ctl( int vic, int slot );
static void VIC_set_vec_ctl( int vic, int slot, u32 bits );
*/

/* Helper functions for working with timers */
static void Tc_init( int id );
static void Tc_cycle( int id, ARMul_State * state );
static u32 Tc_load( int id );
static void Tc_set_load( int id, u32 bits );
static u32 Tc_value( int id );
static u32 Tc_ctl( int id );
static void Tc_set_ctl( int id, u32 bits );

static void Tc4_init();
static void Tc4_cycle( ARMul_State * state );
static u32 Tc4_value_low();
static u32 Tc4_value_high();
static void Tc4_set_high_value( u32 bits );

/*Internal IO Register*/
typedef struct ep9312_io
{
	ARMword syscon_devcfg;	/* System control */

        struct ep9312_vic_io  vic[NR_VIC];
	struct ep9312_tc_io   tc[NR_TC];
	struct ep9312_tc4_io  tc4;
	struct ep9312_uart_io uart[NR_UART];

} ep9312_io_t;

static ep9312_io_t ep9312_io;
#define io ep9312_io

static void
ep9312_update_int (ARMul_State * state)
{
        u32 fiq_request = VIC_fiq_status( 0 ) || VIC_fiq_status( 1 );
        u32 irq_request = VIC_irq_status( 0 ) || VIC_irq_status( 1 );
	state->NfiqSig = fiq_request ? LOW : HIGH;
	state->NirqSig = irq_request ? LOW : HIGH;
}

static void
ep9312_io_reset (ARMul_State * state)
{
	int i;
	io.syscon_devcfg = 0;
        VIC_init( 0 );
        VIC_init( 1 );

        for( i = 0; i < NR_TC; ++i ) {
            Tc_init( i );
        }
        Tc4_init();

	for (i = 0; i < NR_UART; i++) {
		io.uart[i].dr = 0;
		io.uart[i].fr = UART_FR_TXFE;
	}
}


void
ep9312_io_do_cycle (ARMul_State * state)
{
	int i;
	
	/* We implement TC1, TC2, TC3 and TC4 */
	for (i = 0; i < NR_TC; i++) {
            Tc_cycle( i, state );
	}
        Tc4_cycle( state );

	if (!(VIC_raw_int(0) & (UART_RXINTR[iConsole]))
	    && io.uart[iConsole].dr == 0) {
		/* 2007-01-18 modified by Anthony Lee : for new uart device frame */
		struct timeval tv;
		unsigned char buf;

		tv.tv_sec = 0;
		tv.tv_usec = 0;

		if(skyeye_uart_read(-1, &buf, 1, &tv, NULL) > 0)
		{
			io.uart[iConsole].dr = (int) buf;
                        VIC_raise_int( 0, UART_RXINTR[iConsole] );
                        VIC_raise_int( 1, INT_UART[iConsole] );
			io.uart[iConsole].iir |= UART_IIR_RIS;
			io.uart[iConsole].fr &= ~UART_FR_RXFE;
			ep9312_update_int (state);
		}
	}
}


static void
ep9312_uart_read (ARMul_State * state, u32 offset, u32 * data, int index)
{
	switch (offset) {
	case UART_DR:
		*data = io.uart[index].dr;
		io.uart[index].dr = 0;
                VIC_clear_int( 0, UART_RXINTR[index] );
                VIC_clear_int( 1, INT_UART[index] );
		io.uart[index].iir &= ~UART_IIR_RIS;
		io.uart[index].fr |= UART_FR_RXFE;
		extern ARMul_State* state;
		ep9312_update_int (state);
		break;
	case UART_RSR:
		*data = io.uart[index].rsr;
		break;
		//case UART_ECR:
	case UART_CR_H:
	case UART_CR_M:
	case UART_CR_L:
		break;
	case UART_CR:
		*data = io.uart[index].cr;
		break;
	case UART_FR:
		*data = io.uart[index].fr;
		break;
	case UART_IIR:
		*data = io.uart[index].iir;
		break;
		//case UART_ICR:
	case UART_ILPR:
	case UART_DMACR:
	case UART_TCR:
	case UART_TISR:
	case UART_TOCR:
	case UART_TMR:
	case UART_MCR:
	case UART_MSR:
		break;
	default:
		SKYEYE_DBG ("%s(0x%x, 0x%x)\n", __func__, offset, data);
		break;
	}
}
static void
ep9312_uart_write (ARMul_State * state, u32 offset, u32 data, int index)
{
	switch (offset) {
	case UART_DR:
		{
			char c = data;

			/* 2007-01-18 modified by Anthony Lee : for new uart device frame */
			skyeye_uart_write(-1, &c, 1, NULL);
		}
	case UART_RSR:
		//case UART_ECR:
	case UART_CR_H:
	case UART_CR_M:
	case UART_CR_L:
		break;
	case UART_CR:
		{
			io.uart[index].cr = data;
			if ((data & AMBA_UARTCR_TIE) == 0) {
                                VIC_clear_int( 0, UART_RXINTR[index] );
                                VIC_clear_int( 1, INT_UART[index] );

				io.uart[index].iir &= ~(UART_IIR_TIS);	//Interrupt Identification and Clear
			}
			else {

                                VIC_raise_int( 0, UART_RXINTR[iConsole] );
                                VIC_raise_int( 1, INT_UART[iConsole] );
				io.uart[index].iir |= (UART_IIR_TIS);
			}
			extern ARMul_State * state;
			ep9312_update_int (state);
		}
		break;
	case UART_FR:
	case UART_IIR:
		io.uart[index].iir = data;
		break;
		//case UART_ICR:
	case UART_ILPR:
	case UART_DMACR:
	case UART_TCR:
	case UART_TISR:
	case UART_TOCR:
	case UART_TMR:
	case UART_MCR:
	case UART_MSR:
		break;
	default:
		SKYEYE_DBG ("%s(0x%x, 0x%x)\n", __func__, offset, data);
	}
}

/* Timer read/write register 
 */
static void
ep9312_tc_read (u32 offset, u32 * data, int index)
{
	switch (offset) {
	case TC_LOAD:
		*data = Tc_load( index );
		break;
	case TC_VALUE:
		*data = Tc_value( index );
		break;
	case TC_CTL:
		*data = Tc_ctl( index );
		break;
	case TC_CLEAR:
		SKYEYE_DBG ("%s(0x%x, 0x%x): read WO register\n", __func__,
			    offset, data);
		break;
	default:
		SKYEYE_DBG ("%s(0x%x, 0x%x)\n", __func__, offset, data);
		break;
	}
}
static void
ep9312_tc_write (ARMul_State * state, u32 offset, u32 data, int index)
{
	switch (offset) {
	case TC_LOAD:
                Tc_set_load( index, data );
		break;
	case TC_VALUE:
		SKYEYE_DBG ("%s(0x%x, 0x%x): write RO register\n", __func__,
			    offset, data);
		break;
	case TC_CTL:
		Tc_set_ctl( index, data );
		break;
	case TC_CLEAR:
                VIC_clear_int( TCOI[index].vic, TCOI[index].bit );
		extern ARMul_State * state;
		ep9312_update_int (state);
		break;
	default:
		SKYEYE_DBG ("%s(0x%x, 0x%x)\n", __func__, offset, data);
		break;
	}
}

ARMword
ep9312_io_read_byte (ARMul_State * state, ARMword addr)
{
	return ep9312_io_read_word (state, addr);
}

ARMword
ep9312_io_read_halfword (ARMul_State * state, ARMword addr)
{

	SKYEYE_DBG ("SKYEYE: %s error\n", __func__);
}

ARMword
ep9312_io_read_word (ARMul_State * state, ARMword addr)
{
	ARMword data = 0;

	/* TC1 */
	if ((addr >= EP9312_TC_BASE1) &&
	    (addr < (EP9312_TC_BASE1 + EP9312_TC_SIZE))) {
		ep9312_tc_read ((u32) (addr - EP9312_TC_BASE1),
				(u32 *) & data, 0);
            return data;
	}
	/* TC2 */
	if ((addr >= EP9312_TC_BASE2) &&
	    (addr < (EP9312_TC_BASE2 + EP9312_TC_SIZE))) {
		ep9312_tc_read ((u32) (addr - EP9312_TC_BASE2),
				(u32 *) & data, 1);
            return data;
	}
	/* TC3 */
	if ((addr >= EP9312_TC_BASE3) &&
	    (addr < (EP9312_TC_BASE3 + EP9312_TC_SIZE))) {
		ep9312_tc_read ((u32) (addr - EP9312_TC_BASE3),
				(u32 *) & data, 2);
            return data;
	}
	/* UART1 */
	if ((addr >= EP9312_UART_BASE1) &&
	    (addr < (EP9312_UART_BASE1 + EP9312_UART_SIZE))) {
		ep9312_uart_read (state, (u32) (addr - EP9312_UART_BASE1),
				  (u32 *) & data, 0);
	    return data;
	}
	/* UART3 */
	if ((addr >= EP9312_UART_BASE3) &&
	    (addr < (EP9312_UART_BASE3 + EP9312_UART_SIZE))) {
		ep9312_uart_read (state, (u32) (addr - EP9312_UART_BASE3),
				  (u32 *) & data, 2);
	    return data;
	}
	switch (addr) {
	case SYSCON_PWRCNT:
		break;
        case TIMER4VALUELOW:
                data = Tc4_value_low();
                break;
        case TIMER4VALUEHIGH:
                data = Tc4_value_high();
                break;
	case VIC0IRQSTATUS:
		data = VIC_irq_status( 0 );
		break;
	case VIC1IRQSTATUS:
		data = VIC_irq_status( 1 );
		break;
	case VIC0FIQSTATUS:
		data = VIC_fiq_status( 0 );
		break;
	case VIC1FIQSTATUS:
		data = VIC_fiq_status( 1 );
		break;
	case VIC0RAWINTR:
		data = VIC_raw_int( 0 );
		break;
	case VIC1RAWINTR:
		data = VIC_raw_int( 1 );
		break;
        case VIC0INTSELECT:
                data = VIC_int_select( 0 );
                break;
        case VIC1INTSELECT:
                data = VIC_int_select( 1 );
                break;
	case VIC0INTENABLE:
		data = VIC_int_enabled( 0 );
		break;
	case VIC1INTENABLE:
		data = VIC_int_enabled( 1 );
		break;
        case VIC0SOFTINT:
                data = VIC_int_soft_status( 0 );
                break;
        case VIC1SOFTINT:
                data = VIC_int_soft_status( 1 );
                break;
	case RTCDR:
	case AACGCR:
	case AACRGIS:
//              printf("%s(0x%08x) = 0x%08x\n", __func__, addr, data);
		break;
	case SYSCON_DEVCFG:
		data = io.syscon_devcfg;
		break;
	default:
		SKYEYE_DBG ("SKYEYE:unknown io addr, %s(0x%08x) = 0x%08x\n",
			    __func__, addr, data);
		break;
	}
	return data;
}

void
ep9312_io_write_byte (ARMul_State * state, ARMword addr, ARMword data)
{
	ep9312_io_write_word (state, addr, data);
}

void
ep9312_io_write_halfword (ARMul_State * state, ARMword addr, ARMword data)
{
	SKYEYE_DBG ("SKYEYE: %s error\n", __func__);
}

void
ep9312_io_write_word (ARMul_State * state, ARMword addr, ARMword data)
{
	ARMword tmp;
        /* TC1 */
	if ((addr >= EP9312_TC_BASE1) &&
	    (addr < (EP9312_TC_BASE1 + EP9312_TC_SIZE))) {
		ep9312_tc_write (state, (u32) (addr - EP9312_TC_BASE1), data,
				 0);
            return;
	}
        /* TC2 */
	if ((addr >= EP9312_TC_BASE2) &&
	    (addr < (EP9312_TC_BASE2 + EP9312_TC_SIZE))) {
		ep9312_tc_write (state, (u32) (addr - EP9312_TC_BASE2), data,
				 1);
            return;
	}
        /* TC3 */
	if ((addr >= EP9312_TC_BASE3) &&
	    (addr < (EP9312_TC_BASE3 + EP9312_TC_SIZE))) {
		ep9312_tc_write (state, (u32) (addr - EP9312_TC_BASE3), data,
				 2);
            return;
        }
        /* UART1 */
	if ((addr >= EP9312_UART_BASE1) &&
	    (addr < (EP9312_UART_BASE1 + EP9312_UART_SIZE))) {
		ep9312_uart_write (state, (u32) (addr - EP9312_UART_BASE1),
				   data, 0);
            return;
	}
        /* UART3 */
	if ((addr >= EP9312_UART_BASE3) &&
	    (addr < (EP9312_UART_BASE3 + EP9312_UART_SIZE))) {
		ep9312_uart_write (state, (u32) (addr - EP9312_UART_BASE3),
				   data, 2);
            return;
	}

	switch (addr) {
	case SYSCON_CLKSET1:
		break;
	case SYSCON_CLKSET2:
	case SYSCON_PWRCNT:
		break;
        case TIMER4VALUELOW:
                break;
        case TIMER4VALUEHIGH:
                Tc4_set_high_value( data );
                break;
        case VIC0INTSELECT:
                VIC_select_int( 0, data );
		ep9312_update_int (state);
                break;
        case VIC1INTSELECT:
                VIC_select_int( 1, data );
		ep9312_update_int (state);
                break;
	case VIC0INTENABLE:
		VIC_enable_int( 0, data );
		ep9312_update_int (state);
		break;
	case VIC1INTENABLE:
		VIC_enable_int( 1, data );
		ep9312_update_int (state);
		break;
	case VIC0INTENCLEAR:
                VIC_disable_int( 0, data );
		ep9312_update_int (state);
		break;
	case VIC1INTENCLEAR:
                VIC_disable_int( 1, data );
		ep9312_update_int (state);
		break;
        case VIC0SOFTINT:
                VIC_raise_soft_int( 0, data );
		ep9312_update_int (state);
                break;
        case VIC1SOFTINT:
                VIC_raise_soft_int( 1, data );
		ep9312_update_int (state);
                break;
        case VIC0SOFTINTCLEAR:
                VIC_clear_soft_int( 0, data );
		ep9312_update_int (state);
                break;
        case VIC1SOFTINTCLEAR:
                VIC_clear_soft_int( 1, data );
		ep9312_update_int (state);
                break;
	case SYSCON_DEVCFG:
		io.syscon_devcfg = data;
		break;
	default:
		SKYEYE_DBG
			("SKYEYE:unknown io addr, %s(0x%08x, 0x%08x), pc %x \n",
			 __func__, addr, data, state->Reg[15]);
		break;
	}
}

void
ep9312_mach_init (void* arch_instance, machine_config_t * this_mach)
{
	extern ARMul_State * state;
	ARMul_SelectProcessor (state, ARM_v4_Prop);
	/* ARM920T uses LOW */
	state->lateabtSig = LOW;

//      state->Reg[1] = 282;    //for EP9312 2.4.x arch id
	state->Reg[1] = 451;	//for EP9312 2.6.x arch id
	//state->Reg[1] = 386;  //for EP9315 2.4.x arch id
	this_mach->mach_io_do_cycle = ep9312_io_do_cycle;
	this_mach->mach_io_reset = ep9312_io_reset;
	this_mach->mach_io_read_byte = ep9312_io_read_byte;
	this_mach->mach_io_write_byte = ep9312_io_write_byte;
	this_mach->mach_io_read_halfword = ep9312_io_read_halfword;
	this_mach->mach_io_write_halfword = ep9312_io_write_halfword;
	this_mach->mach_io_read_word = ep9312_io_read_word;
	this_mach->mach_io_write_word = ep9312_io_write_word;

	this_mach->mach_update_int = ep9312_update_int;

}

static void VIC_init( int vic )
{
    io.vic[vic].int_status = 0;
    io.vic[vic].int_mask = 0;
    io.vic[vic].int_mode = 0;
    io.vic[vic].int_soft = 0;
}
static u32 VIC_irq_status( int vic )
{
    return (io.vic[vic].int_status | io.vic[vic].int_soft) & 
           io.vic[vic].int_mask & 
           ~io.vic[vic].int_mode;
}
static u32 VIC_fiq_status( int vic )
{
    return (io.vic[vic].int_status | io.vic[vic].int_soft) & 
           io.vic[vic].int_mask & 
           io.vic[vic].int_mode;
}
static u32 VIC_int_select( int vic )
{
    return io.vic[vic].int_mode;
}
static void VIC_select_int( int vic, u32 bits )
{
    io.vic[vic].int_mode = bits;
}
static u32 VIC_int_enabled( int vic )
{
    return io.vic[vic].int_mask;
}
static void VIC_enable_int( int vic, u32 bits )
{
    io.vic[vic].int_mask |= bits;
}
static void VIC_disable_int( int vic, u32 bits )
{
    io.vic[vic].int_mask &= ~bits;
}
static u32 VIC_raw_int( int vic )
{
    return io.vic[vic].int_status | io.vic[vic].int_soft;
}
static void VIC_raise_int( int vic, u32 bits )
{
    io.vic[vic].int_status |= bits;
}
static void VIC_clear_int( int vic, u32 bits )
{
    io.vic[vic].int_status &= ~bits;
}
static u32 VIC_int_soft_status( int vic )
{
    return io.vic[vic].int_soft;
}
static void VIC_raise_soft_int( int vic, u32 bits )
{
    io.vic[vic].int_soft |= bits;
}
static void VIC_clear_soft_int( int vic, u32 bits )
{
    io.vic[vic].int_soft &= ~bits;
}

#define EP9312_TC_COUNTER_LOAD_0 EP9312_HCLK_FREQ / EP9312_TC_FREQ_0
#define EP9312_TC_COUNTER_LOAD_1 EP9312_HCLK_FREQ / EP9312_TC_FREQ_1
#define EP9312_TC4_COUNTER_LOAD  EP9312_HCLK_FREQ / EP9312_TC4_FREQ

static void Tc_init( int id )
{
    io.tc[id].load = 0;
    io.tc[id].value = 0;
    io.tc[id].ctl = 0;
    switch( id ) {
        case 0:
            io.tc[id].mod_value = EP9312_TC16_MOD;
            break;
        case 1:
            io.tc[id].mod_value = EP9312_TC16_MOD;
            break;
        case 2:
            io.tc[id].mod_value = EP9312_TC32_MOD;
            break;
        default:
            break;
    }
    io.tc[id].counter = 0;
}
static void Tc_cycle( int id, ARMul_State * state )
{
    if( io.tc[id].ctl & TC_CTL_ENABLE ) {
        if (io.tc[id].value == 0) {
            if (io.tc[id].ctl & TC_CTL_MODE)
                io.tc[id].value = io.tc[id].load;
            else
                io.tc[id].value = io.tc[id].mod_value;
            VIC_raise_int( TCOI[id].vic, TCOI[id].bit );
            ep9312_update_int (state);
        }
        else {
            if( io.tc[id].counter == 0 ) {
                io.tc[id].value--;
                io.tc[id].counter = 
                    ( io.tc[id].ctl & TC_CTL_CLKSEL ) ?
                        EP9312_TC_COUNTER_LOAD_0 :
                        EP9312_TC_COUNTER_LOAD_1;
            } else {
                io.tc[id].counter--;
            }
        }
    }
}
static u32 Tc_load( int id )
{
    return io.tc[id].load;
}
static void Tc_set_load( int id, u32 bits )
{
    io.tc[id].load = bits;
    io.tc[id].value = bits;
}
static u32 Tc_value( int id )
{
    return io.tc[id].value;
}
static u32 Tc_ctl( int id )
{
    return io.tc[id].ctl;
}
static void Tc_set_ctl( int id, u32 bits )
{
    io.tc[id].ctl = bits;
}

static void Tc4_init()
{
    io.tc4.value = 0;
    io.tc4.counter = 0;
}
static void Tc4_cycle( ARMul_State * state )
{
    if( io.tc4.value & EP9312_TC_ENABLE_BIT ) {
        if( io.tc4.counter == 0 ) {
            io.tc4.value++;
            io.tc4.counter = EP9312_TC4_COUNTER_LOAD;
        } else {
            io.tc4.counter--;
        }
    }
}
static u32 Tc4_value_low()
{
    return (u32)io.tc4.value;
}
static u32 Tc4_value_high()
{
    return (u32)(io.tc4.value >> 32);
}
static void Tc4_set_high_value( u32 bits )
{
    io.tc4.value = (((u64)bits) << 32) & EP9312_TC_ENABLE_BIT |
                   io.tc4.value & ~EP9312_TC_ENABLE_BIT;
    if( (io.tc4.value & EP9312_TC_ENABLE_BIT) == 0 ) {
        io.tc4.value = 0;
        io.tc4.counter = 0;
    }
}
