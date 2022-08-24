#!/usr/bin/env python3

#
# This file is part of LiteEth.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2022 Roman Pechenko <romanetz4@gmail.com>
# Copyright (c) 2022 Fabio Eboli <faeboli@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse
import platform
import time

from migen import *

from litex_boards.platforms import colorlight_5a_75b, colorlight_5a_75e
from litex_boards.targets.colorlight_5a_75x import _CRG

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.cores.pwm import PWM
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.spi_flash import ECP5SPIFlash
from litex.soc.cores.gpio import GPIOOut,GPIOIn
from litex.soc.cores.led import LedChaser

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from litex.build.generic_platform import *
from litex.soc.interconnect.csr import *
from liteeth.frontend.stream import *
from liteeth.core import LiteEthUDPIPCore
from liteeth.common import *

# Devices configuration start ----------------------------------------------------------------------------------------
encoders=([
    ("encoder", 0,
     Subsignal("A", Pins("j1:0")), # J9.1
     Subsignal("B", Pins("j1:1")), # J9.2
 #    Subsignal("Z", Pins("j7:2")), # false
     IOStandard("LVCMOS33")),
    ("encoder", 1,
     Subsignal("A", Pins("j1:2")),  # J9.5
     Subsignal("B", Pins("j1:4")),  # J9.6
  #   Subsignal("Z", Pins("j7:6")),  # false
     IOStandard("LVCMOS33")
     ),
    ("encoder", 2,
     Subsignal("A", Pins("j1:5")),  # J10.1
     Subsignal("B", Pins("j1:6")),  # J10.2
#     Subsignal("Z", Pins("j8:2")),  # false
     IOStandard("LVCMOS33")
     ),
    ("encoder", 3,
     Subsignal("A", Pins("j2:0")),  # J10.5
     Subsignal("B", Pins("j2:1")),  # J10.6
#     Subsignal("Z", Pins("j8:6")),  # false
     IOStandard("LVCMOS33")
     ),
    ("encoder", 4,
     Subsignal("A", Pins("j2:2")),  # J11.1
     Subsignal("B", Pins("j2:4")),  # J11.2
#     Subsignal("Z", Pins("j1:14")),  # false
     IOStandard("LVCMOS33")
     ),
    ("encoder", 5,
     Subsignal("A", Pins("j2:5")),  # J11.5
     Subsignal("B", Pins("j2:6")),  # J11.6
 #    Subsignal("Z", Pins("j1:14")),  # false
     IOStandard("LVCMOS33")
     ),
])

stepgens=([
    ("stepgen", 0,
     Subsignal("step", Pins("j9:0")), # J1.1
     Subsignal("dir", Pins("j9:1")), # J1.2
     IOStandard("LVCMOS33")),
    ("stepgen", 1,
     Subsignal("step", Pins("j9:2")),  # J1.3
     Subsignal("dir", Pins("j9:4")),  # J1.5
     IOStandard("LVCMOS33")
     ),
    ("stepgen", 2,
     Subsignal("step", Pins("j9:5")),  # J1.6
     Subsignal("dir", Pins("j9:6")),  # J1.7
     IOStandard("LVCMOS33")
     ),
    ("stepgen", 3,
     Subsignal("step", Pins("j10:0")),  # J2.1
     Subsignal("dir", Pins("j10:1")),  # J2.2
     IOStandard("LVCMOS33")
     ),
    ("stepgen", 4,
     Subsignal("step", Pins("j10:2")),  # J2.3
     Subsignal("dir", Pins("j10:4")),  # J2.5
     IOStandard("LVCMOS33")
     ),
    ("stepgen", 5,
     Subsignal("step", Pins("j10:5")),  # J2.6
     Subsignal("dir", Pins("j10:6")),  # J2.7
     IOStandard("LVCMOS33")
     ),
])
#gpios in
_gpios_in = [
    ("gpio_in", 0, Pins("j3:0"), IOStandard("LVCMOS33")),
    ("gpio_in", 1, Pins("j3:1"), IOStandard("LVCMOS33")),
    ("gpio_in", 2, Pins("j3:2"), IOStandard("LVCMOS33")),
    ("gpio_in", 3, Pins("j3:4"), IOStandard("LVCMOS33")),
    ("gpio_in", 4, Pins("j3:5"), IOStandard("LVCMOS33")),
    ("gpio_in", 5, Pins("j3:6"), IOStandard("LVCMOS33")),
    ("gpio_in", 6, Pins("j4:1"), IOStandard("LVCMOS33")),
    ("gpio_in", 7, Pins("j4:2"), IOStandard("LVCMOS33")),
    ("gpio_in", 8, Pins("j4:4"), IOStandard("LVCMOS33")),
    ("gpio_in", 9, Pins("j4:5"), IOStandard("LVCMOS33")),
    ("gpio_in", 10, Pins("j4:6"), IOStandard("LVCMOS33")),
]
#gpios out
_gpios_out = [ \
    ("gpio_out", 0, Pins("j11:1"), IOStandard("LVCMOS33")),
    ("gpio_out", 1, Pins("j11:2"), IOStandard("LVCMOS33")),
    ("gpio_out", 2, Pins("j11:5"), IOStandard("LVCMOS33")),
    ("gpio_out", 3, Pins("j11:6"), IOStandard("LVCMOS33")),
    ("gpio_out", 4, Pins("j12:0"), IOStandard("LVCMOS33")),
    ("gpio_out", 5, Pins("j12:1"), IOStandard("LVCMOS33")),
    ("gpio_out", 6, Pins("j12:2"), IOStandard("LVCMOS33")),
    ("gpio_out", 7, Pins("j12:4"), IOStandard("LVCMOS33")),
    ("gpio_out", 8, Pins("j12:5"), IOStandard("LVCMOS33")),
    ("gpio_out", 9, Pins("j12:6"), IOStandard("LVCMOS33")),
]

_pwm_out = [ \
    ("pwm_out", 0, Pins("j11:0"), IOStandard("LVCMOS33")),
    ("pwm_out", 1, Pins("j11:4"), IOStandard("LVCMOS33")),
]

_ext_reset_in = [("ext_reset_in", 0, Pins("j4:0"), IOStandard("LVCMOS33"))]

# Devices configuration end ----------------------------------------------------------------------------------------

#watchdog register setup
watchdog_size=22
watchdog_offs=10

# global for number of each device
num_inputs=len(_gpios_in)
num_outputs=len(_gpios_out)
num_encoders=len(encoders)
num_pwm=len(_pwm_out)
num_stepgens=len(stepgens)

class QuadEnc(Module,AutoCSR):
    def __init__(self, pads):
        self.pads = pads
        #physical pads
        a=Signal() #input A
        b=Signal() #input B
        
        #register outputs
        self.out=Signal(32) #counter output
        #register inputs
        self.reset=Signal(1) #reset counter
        self.enable=Signal(1) #enable counter
        #Internal signals
        syncr=Signal(2) #Syncronization register
        AB=Signal(2) #Syncronization register
        os=Signal(2) #old state
        ns=Signal(2) #new state
        tmp=Signal(2) #

        self.comb += a.eq(pads.A)
        self.comb += b.eq(pads.B)
        self.sync+=If(self.enable==1,              # first syncronizer
        syncr.eq(Cat(a,b))) 
        self.sync+=AB.eq(syncr)                      # second syncronizer
        self.comb+=tmp.eq(Cat(AB[1]^AB[0],AB[1]))
        self.comb+=ns.eq(tmp-os)
        self.sync+=If(ns[0]==1,(os.eq(os+ns)),(If(ns[1]==1,self.out.eq(self.out-1)).Else(self.out.eq(self.out+1))))
        self.sync+=If(self.reset==1,
        self.out.eq(0))

class StepGen(Module,AutoCSR):
    def __init__(self, pads):
        self.pads = pads
        #physical pads
        step=Signal()# step out        
        sdir=Signal()# dir out

        #register inputs
        self.velocity=Signal(32) #frequency input
        self.sign=Signal(1) #frequency sign
        self.reset=Signal(1) #reset 
        self.enable=Signal(1) #enable
        self.inv_step=Signal(1) #invert step pin
        self.inv_dir=Signal(1) #invert direction pin
        self.step_width=Signal(9)# step minimum width
        self.dir_width=Signal(9)# dir minimum width
        self.dir_setuptime=Signal(14)#dir minimum distance dir-step and step-dir
        
        #register outputs
        self.position_fb=Signal(32) #position output
        
        #internal signals
#        int_step=Signal(1)
#        int_dir=Signal(1)
        counter=Signal(33)#internal counter
        step_tmr=Signal(9)# step width timer
        dir_tmr=Signal(9)# dir min width timer
        dir_setuptimer=Signal(14)#dir setup timer
        dir_old=Signal(1)# dir out
        
        #pads
#        self.comb+=pads.step.eq(step)
#        self.comb+=pads.dir.eq(sdir)

        self.comb+=If(self.inv_step==1,		#Evaluate step inversion
        pads.step.eq(~step)).Else(pads.step.eq(step))
        self.comb+=If(self.inv_dir==1,		#Evaluate dir inversion
        pads.dir.eq(~sdir)).Else(pads.dir.eq(sdir))
        
        self.sync+=If(self.enable==1,               # increment counter with frequency term
        counter.eq(counter+self.velocity))
        
        self.sync+=If(counter[32]==1,       # use carry for output
        counter[32].eq(0),                           # reset carry
        step_tmr.eq(self.step_width), #,    # initialize timer for minimum step duration
        If(sdir==0,                                        # update the position counter
        self.position_fb.eq(self.position_fb+1)
        ).Else(
        self.position_fb.eq(self.position_fb-1)))
        
        self.sync+=If(step_tmr>0,           # decrement timer for minimum step duration
        step_tmr.eq(step_tmr-1),
        step.eq(1)).Else(step.eq(0)) # emit step pulse
        
        self.sync+=If(step==1,                 # initialize setup timer for step to dir time
        If(step_tmr==0,
        dir_setuptimer.eq(self.dir_setuptime)))
        
        self.sync+=If(dir_setuptimer>0, # decrement setup timer
        dir_setuptimer.eq(dir_setuptimer-1))
        
        self.sync+=If(dir_setuptimer==0, # update dir only after setup time from last step end
        If( dir_tmr==0,                                 # and if minimum dir time is passed
        dir_old.eq(sdir),
        sdir.eq(self.sign)))
        
        self.sync+=If(dir_old!=sdir,    # initialize timer for minimum dir duration
        dir_tmr.eq(self.dir_width))
        
        self.sync+=If(dir_tmr>0,              # time for minimum dir duration update
        dir_tmr.eq(dir_tmr-1))
        
        self.sync+=If(self.reset==1,
        counter.eq(0),
        step_tmr.eq(0),
        dir_tmr.eq(0),
        dir_setuptimer.eq(0),
        self.position_fb.eq(0),
        dir_old.eq(0))

class MMIO(Module,AutoCSR):
    def __init__(self):
        self.wallclock = Signal(32) #implement overflow

        for i in range(num_stepgens):
           setattr(self,f'velocity{i}', CSRStorage(size=32, description="Stepgen velocity", write_from_dev=False, name='velocity_'+str(i)))

        self.step_sign = CSRStorage(fields=[
        CSRField("sign", size=16, offset=0,description="Velocity sign")],
        description="Stepgen velocity sign", write_from_dev=False, name='stepsign')
        self.step_res_en = CSRStorage(fields=[
        CSRField("sgreset", size=16, offset=0,description="Reset"),
        CSRField("sgenable", size=16, offset=16,description="Enable")],
        description="Stepgen Enable and Reset flags", write_from_dev=False)
        self.step_dir_inv = CSRStorage(fields=[
        CSRField("dir_inv", size=16, offset=0,description="Dir Pin Inversion"),
        CSRField("step_inv", size=16, offset=16,description="Step Pin Inversion")],
        description="Stepgen Dir and Step inversion", write_from_dev=False, name='stepdirinv')
        self.steptimes = CSRStorage(fields=[
        CSRField("dir_setup", size=14, offset=0,description="Dir Pin Setup time"),
        CSRField("dir_width", size=9, offset=14,description="Dir Pin Minimum width"),
        CSRField("step_width", size=9, offset=23,description="Step Pin Minimum width")],
        description="Stepgen steptime", write_from_dev=False, name='steptimes')

        self.gpios_out = CSRStorage(size=32, description="gpios out", write_from_dev=False, name='gpios_out')

        for i in range(num_pwm):
            setattr(self,f'pwm{i}', CSRStorage(fields=[
        CSRField("width", size=16, offset=0,description="PWM Width"),
        CSRField("period", size=16, offset=16,description="PWM Period")],
        description="PWM width and period", write_from_dev=False, name='pwm_'+str(i)))
        
        self.enc_res_en = CSRStorage(fields=[
        CSRField("reset", size=16, offset=0,description="Reset"),
        CSRField("enable", size=16, offset=16,description="Enable")],
        description="Encoder enable and reset flags", write_from_dev=False)        
        
        self.res_st_reg= CSRStorage(fields=[
        CSRField("watchdog", size=watchdog_size, offset=watchdog_offs, access=2, description="watchdog down counter")],
        description="Reset and status register",write_from_dev=True, name='res_st_reg')

        for i in range(num_stepgens):        
            setattr(self,f'sg_count{i}', CSRStatus(size=32, description="Stepgen "+str(i)+" count", name="sg_count_"+str(i)))
                    
        self.wallclock = CSRStatus(size=32, description="wallclock time", name='wallclock')
        self.gpios_in = CSRStatus(size=32, description="gpios in", name='gpios_in')

        for i in range(num_encoders):
            setattr(self,f'enc_count{i}', CSRStatus(size=32, description="Encoder "+str(i)+" count", name="enc_count_"+str(i)))

def _to_signal(obj):
    return obj.raw_bits() if isinstance(obj, Record) else obj

class BaseSoC(SoCMini): 
    def __init__(self, board, revision,
                          sys_clk_freq=int(50e6),
                          mac_address=0x10e2d5000000, 
                          ip_address="192.168.1.50",
                          eth_phy=0, **kwargs):
        
        #external reset from pins
        external_reset = Signal(1)
        #global reset from watchdog
        global_reset = Signal(1)
        #watchdog prescaler
        wdt_psc = Signal(6)
        #watchdog counter
        wdt_count= Signal(watchdog_size)

        board = board.lower()
        assert board in ["5a-75b", "5a-75e"]
        if board == "5a-75b":
            platform = colorlight_5a_75b.Platform(revision=revision)
        elif board == "5a-75e":
            platform = colorlight_5a_75e.Platform(revision=revision)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq, ident="Lcnc " + board.upper())

        platform.add_extension(encoders)
        platform.add_extension(stepgens)
	
        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq, with_rst = False)        

        # Etherbone --------------------------------------------------------------------------------
        self.submodules.ethphy = LiteEthPHYRGMII(
            clock_pads = self.platform.request("eth_clocks",eth_phy),
            pads       = self.platform.request("eth",eth_phy),
            tx_delay   = 0e-9,
            with_hw_init_reset = False)

        self.add_etherbone(phy=self.ethphy,
            buffer_depth=255,
            mac_address=mac_address,
            ip_address=ip_address)

        platform.add_extension(_ext_reset_in) 		        
        platform.add_extension(_gpios_in)
        platform.add_extension(_gpios_out)
        platform.add_extension(_pwm_out)
        
        self.ext_reset_in = platform.request("ext_reset_in")
        self.gpio_in_pads  = platform.request_all("gpio_in")
        self.gpio_out_pads  = platform.request_all("gpio_out")
        self.pwm_out_pads  = platform.request_all("pwm_out")

        self.gpio_inputs = [(self.gpio_in_pads.l[i]) for i in range(num_inputs)]
        self.gpio_outputs = [(self.gpio_out_pads.l[i]) for i in range(num_outputs)]

        for i in range(num_encoders):
            setattr(self.submodules,f'encoder{i}', QuadEnc(platform.request("encoder", i)))

        for i in range(num_stepgens):
            setattr(self.submodules,f'stepgen{i}',StepGen(platform.request("stepgen", i)))

        for i in range(num_pwm):
            setattr(self.submodules,f'pwm{i}', PWM(pwm=self.pwm_out_pads.l[i], default_enable=True, default_width=16, default_period=16, with_csr=False))

        self.submodules.MMIO_inst = MMIO_inst = MMIO()
		
        for i in range(num_stepgens):
            self.sync+=[
            getattr(self.MMIO_inst,f'sg_count{i}').status.eq(getattr(self,f'stepgen{i}').position_fb),
            getattr(self.MMIO_inst,f'sg_count{i}').we.eq(True),
            getattr(self,f'stepgen{i}').enable.eq(self.MMIO_inst.step_res_en.fields.sgenable[i]),
            getattr(self,f'stepgen{i}').reset.eq(self.MMIO_inst.step_res_en.fields.sgreset[i] | global_reset),
            getattr(self,f'stepgen{i}').sign.eq(self.MMIO_inst.step_sign.fields.sign[i]),
            getattr(self,f'stepgen{i}').inv_step.eq(self.MMIO_inst.step_dir_inv.fields.step_inv[i]),
            getattr(self,f'stepgen{i}').inv_dir.eq(self.MMIO_inst.step_dir_inv.fields.dir_inv[i]),
            getattr(self,f'stepgen{i}').step_width.eq(self.MMIO_inst.steptimes.fields.step_width),
            getattr(self,f'stepgen{i}').dir_width.eq(self.MMIO_inst.steptimes.fields.dir_width),
            getattr(self,f'stepgen{i}').dir_setuptime.eq(self.MMIO_inst.steptimes.fields.dir_setup),
            getattr(self,f'stepgen{i}').velocity.eq(getattr(self.MMIO_inst,f'velocity{i}').storage)]

        for i in range(num_encoders):
            self.sync+=[
            getattr(self.MMIO_inst,f'enc_count{i}').status.eq(getattr(self,f'encoder{i}').out),
            getattr(self.MMIO_inst,f'enc_count{i}').we.eq(True),
            getattr(self,f'encoder{i}').enable.eq(self.MMIO_inst.enc_res_en.fields.enable[i]),
            getattr(self,f'encoder{i}').reset.eq(self.MMIO_inst.enc_res_en.fields.reset[i] | global_reset)]

        for i in range(num_pwm):
            self.sync+=[
            If(global_reset==True,
            getattr(self,f'pwm{i}').width.eq(0)).Else(
            getattr(self,f'pwm{i}').width.eq(getattr(self.MMIO_inst,f'pwm{i}').fields.width)),
            getattr(self,f'pwm{i}').period.eq(getattr(self.MMIO_inst,f'pwm{i}').fields.period)]

        for i in range(num_outputs):
            self.sync+=[
			If(global_reset==True,
			self.gpio_outputs[i].eq(False)).Else(
			self.gpio_outputs[i].eq(self.MMIO_inst.gpios_out.storage[i]))]
        
        for i in range(num_inputs):
            self.sync+=[self.MMIO_inst.gpios_in.status[i].eq(self.gpio_inputs[i])]
        self.sync+=[self.MMIO_inst.gpios_in.we.eq(True)]
        self.sync+=[self.MMIO_inst.wallclock.status.eq(self.MMIO_inst.wallclock.status+1)]
        
        # watchdog prescaler is a 6-bit freerun counter
        self.sync+=[wdt_psc.eq(wdt_psc+1)]
        #watchdog counter is downcounting to zero
        self.sync+=[self.MMIO_inst.res_st_reg.dat_w[watchdog_offs:watchdog_offs+watchdog_size].eq(wdt_count),self.MMIO_inst.res_st_reg.we.eq(True)]
        self.sync+=[If(self.MMIO_inst.res_st_reg.re == True, wdt_count.eq(self.MMIO_inst.res_st_reg.fields.watchdog))]
        #if external reset is tue the watchdog is reset, if watchdog is zero, reset all peripherals
        self.sync+=[If(external_reset==True,wdt_count.eq(0),global_reset.eq(True)).Elif(wdt_count==0,global_reset.eq(True)).Else(global_reset.eq(False),If(wdt_psc==0,wdt_count.eq(wdt_count-1)))]
        #connect external reset
        self.comb+=external_reset.eq(self.ext_reset_in)
 
# Build ---------------------------------------------------------------------------------------------

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="Lcnc")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--build",             action="store_true",              help="Build design.")
    target_group.add_argument("--load",              action="store_true",              help="Load bitstream.")
    target_group.add_argument("--board",             default="5a-75e",                 help="Board type (5a-75b or 5a-75e).")
    target_group.add_argument("--revision",          default="6.0", type=str,          help="Board revision (6.0, 6.1, 7.0 or 8.0).")
    target_group.add_argument("--sys-clk-freq",      default=50e6,                     help="System clock frequency")
    target_group.add_argument("--eth-ip",            default="192.168.2.50", type=str, help="Ethernet/Etherbone IP address.")
    target_group.add_argument("--eth-phy",           default=0, type=int,              help="Ethernet PHY (0 or 1).")
    target_group.add_argument("--mac-address",           default="0x10e2d5000000",              help="Ethernet MAC address in hex format")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()
  
    num_inputs=len(_gpios_in)
    num_outputs=len(_gpios_out)
    num_encoders=len(encoders)
    num_pwm=len(_pwm_out)
    num_stepgens=len(stepgens)
    print('Num inputs=  ',str(num_inputs))
    print('Num outputs=  ',str(num_outputs))
    print('Num encoders=  ',str(num_encoders))
    print('Num pwm=  ',str(num_pwm))
    print('Num stepgens=  ',str(num_stepgens))

    assert num_encoders < 16 , "Maximum number of encoders is 16 due to MMIO register definition"
    assert num_stepgens < 16 , "Maximum number of stepgens is 16 due to MMIO register definition"
    assert num_inputs < 32 , "Maximum number of inputs is 32 due to MMIO register definition"
    assert num_outputs < 32 , "Maximum number of outputs is 32 due to MMIO register definition"

    #generate driver header
    f=open("configuration_auto.h","w")
    f.write("//---Board configuration---\n")
    f.write("#define IP_ADDR  \""+args.eth_ip+"\"\n")
    f.write("#define UDP_PORT \"1234\"\n")
    f.write("#define F_FPGA ("+str(args.sys_clk_freq)+")\n")
    f.write("//---Devices configuration---\n")
    f.write("#define N_INPUTS "+str(num_inputs)+"\n")
    f.write("#define N_OUTPUTS "+str(num_outputs)+"\n")
    f.write("#define N_ENCODERS "+str(num_encoders)+"\n")
    f.write("#define N_PWM "+str(num_pwm)+"\n")
    f.write("#define N_STEPGENS "+str(num_stepgens)+"\n")
    f.write("//---Registers definition, all are 32bit wide---\n")
    f.write("// --transmitted registers: board to fpga\n")
    f.write("// - start of STEPGEN related regs\n")
    f.write("//  - velocity registers, one per stepgen\n")
    for i in range(num_stepgens):
         if i==0:
            f.write("#define TX_POS_VELOCITY"+str(i)+" 0\n")
         else:
            f.write("#define TX_POS_VELOCITY"+str(i)+" (TX_POS_VELOCITY"+str(i-1)+" + 4)\n")
    f.write("//  - other registers, only one each needed, max 16 STEPGENS allowed\n")
    f.write("#define TX_POS_STEPSIGN (TX_POS_VELOCITY"+str(num_stepgens-1)+" + 4)\n")
    f.write("\
#define TX_POS_STEP_RES_EN (TX_POS_STEPSIGN + 4)\n\
#define TX_POS_STEPDIRINV (TX_POS_STEP_RES_EN + 4)\n\
#define TX_POS_STEPTIMES (TX_POS_STEPDIRINV + 4)\n\
// - end of STEPGEN related regs\n\
// OUTPUTS, only one register needed, max 32 outputs allowed\n\
#define TX_POS_GPIOS_OUT (TX_POS_STEPTIMES + 4)\n")
    f.write("// PWM related\n")
    f.write("//   - pwm control registers, one per PWM\n")
    for i in range(num_pwm):
         if i==0:
            f.write("#define TX_POS_PWM"+str(i)+" (TX_POS_GPIOS_OUT + 4)\n")
         else:
            f.write("#define TX_POS_PWM"+str(i)+" (TX_POS_PWM"+str(i-1)+" + 4)\n")
    f.write("// end of PWM related regs\n")
    f.write("// Encoder control, one register needed, maximum 16 encoders allowed\n")
    f.write("#define TX_POS_ENC_RES_EN (TX_POS_PWM"+str(num_pwm-1)+" + 4)\n")
    f.write("\
// Board control and reset register, containing watchdog, only one needed\n\
#define TX_POS_RES_STAT_REG (TX_POS_ENC_RES_EN + 4)\n\
// number of regiters to be sent to fpga, sum of all the ones so far\n\
#define TX_PAYLOAD_SIZE (TX_POS_RES_STAT_REG + 4)\n")

    f.write("// received register: fpga to board\n")
    f.write("// Board control and reset register, containing watchdog, only one needed\n")
    f.write("#define RX_POS_RES_ST_REG 0\n")
    f.write("// end of ENCODER related regs\n")
    f.write("//  - step counters feedback, one per stepgen\n")
    for i in range(num_stepgens):
         if i==0:
            f.write("#define RX_POS_STEPCOUNT"+str(i)+" (RX_POS_RES_ST_REG + 4)\n")
         else:
            f.write("#define RX_POS_STEPCOUNT"+str(i)+" (RX_POS_STEPCOUNT"+str(i-1)+" + 4)\n")
    f.write("// end of STEPGEN related regs\n\
// WALLCLOCK feedback register, only one needed\n")
    f.write("#define RX_POS_WALLCLOCK (RX_POS_STEPCOUNT"+str(num_stepgens-1)+" + 4)\n")
    f.write("\
// INPUTS register, only one needed, max 32 inputs allowed\n\
#define RX_POS_GPIOS_IN (RX_POS_WALLCLOCK + 4)\n\
// ENCODER related:\n")
    f.write("//  - encoder counters feedback, one per ecoder\n")
    for i in range(num_encoders):
         if i==0:
            f.write("#define RX_POS_ENCCOUNT"+str(i)+" (RX_POS_GPIOS_IN + 4)\n")
         else:
            f.write("#define RX_POS_ENCCOUNT"+str(i)+" (RX_POS_ENCCOUNT"+str(i-1)+" + 4)\n")
    f.write("// end of ENCODER related regs\n\
// number of registers to be sent read from fpga, sum of all the ones so far\n")
    f.write("#define RX_PAYLOAD_SIZE (RX_POS_ENCCOUNT"+str(num_encoders-1)+" + 4)\n")
    f.write("// ---end of configuration\n")

    f.close()
        
    soc = BaseSoC(board=args.board, revision=args.revision,
        sys_clk_freq     = int(float(args.sys_clk_freq)),
        mac_address   = int(args.mac_address,16),
        ip_address           = args.eth_ip,
        eth_phy          = args.eth_phy,
        **soc_core_argdict(args)
    )
    builder = Builder(soc, **builder_argdict(args))
    
    if args.build:
        builder.build(build_name="Lcnc",**trellis_argdict(args))
        
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".svf")) # FIXME

if __name__ == "__main__":
    main()
