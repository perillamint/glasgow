from amaranth import *
from amaranth.sim import Simulator, Delay
from amaranth.lib.data import Struct, Layout

from ... import *
from . import IsaApplet, IsaSubtarget

class IsaAppletTestCase(GlasgowAppletTestCase, applet = IsaApplet):
    def strobe(self, negate, signal, duration):
        yield signal.eq(~negate)
        yield Delay(duration)
        yield signal.eq(negate)
        yield Delay(duration)

    def setup_applet(self):
        self.build_simulated_applet()
        mux_iface = self.applet.mux_interface
        ports = mux_iface._subtargets[0].ports
        in_fifo = mux_iface._subtargets[0].in_fifo
        out_fifo = mux_iface._subtargets[0].out_fifo

        self.ports = ports
        m = Module()

        self.addr = addr = Signal(3)
        self.data_i = data_i = Signal(8)
        self.data_o = data_o = Signal(8)
        self.cs = cs = Signal(1)
        self.ior = ior = Signal(1)
        self.iow = iow = Signal(1)
        self.in_fifo_data = in_fifo_data = Signal(8)
        self.in_fifo_level = in_fifo_level = Signal(8)

        m.d.comb += [
            ports.addr.oe.eq(0b111),
            ports.addr.i.eq(addr),
            ports.data.i.eq(data_i),
            data_o.eq(ports.data.o),
            ports.cs.oe.eq(0b1),
            ports.cs.i.eq(cs),
            ports.ior.oe.eq(0b1),
            ports.ior.i.eq(ior),
            ports.iow.oe.eq(0b1),
            ports.iow.i.eq(iow),
            ports.irq.oe.eq(0b0),
            ports.irq2.oe.eq(0b0),

            in_fifo_data.eq(in_fifo.r_data),
            in_fifo_level.eq(in_fifo.r_level),
        ]

        #m.d.sync += [
        #]
    
        self.target.add_submodule(m)

    @applet_simulation_test("setup_applet")
    def test_host_read(self):

        ## Initial configuration
        yield self.addr.eq(0b000)
        yield self.cs.eq(0)
        yield self.ior.eq(1)
        yield self.iow.eq(1)
        yield Delay(2e-9)

        ## Select the chip
        yield self.cs.eq(1)
        yield Delay(2e-9)
        
        ## Set the address
        yield self.addr.eq(0b001)

        ## Strobe IOR
        yield from self.strobe(True, self.ior, 10e-9)

        yield self.cs.eq(0)

        yield Delay(1e-6)

        #isa_iface = yield self.run_simulated_applet()

    @applet_simulation_test("setup_applet")
    def test_host_write(self):

        ## Initial configuration
        yield self.addr.eq(0b000)
        yield self.cs.eq(0)
        yield self.ior.eq(1)
        yield self.iow.eq(1)
        yield Delay(2e-9)

        ## Select the chip
        yield self.cs.eq(1)
        yield Delay(2e-9)
        
        ## Set the address and data
        yield self.addr.eq(0b001)
        yield self.data_i.eq(0b10101010)

        ## Strobe IOW
        yield from self.strobe(True, self.iow, 10e-9)

        yield self.cs.eq(0)

        yield Delay(1e-6)

        #isa_iface = yield self.run_simulated_applet()