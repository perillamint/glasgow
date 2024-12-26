import logging
import asyncio

from amaranth import *
from amaranth.lib import io, enum, wiring
from amaranth.build.res import ResourceError
from glasgow.access.direct.arguments import PinArgument

from ... import *

__all__ = ["IsaSubtaret", "IsaInterface", "IsaApplet"]

class IsaSubtarget(Elaboratable):
    class Command(enum.Enum):
        NOP = 0x00
        READ = 0x01
        WRITE = 0x02
        READ_TRACE = 0x03
        WRITE_TRACE = 0x04

    def __init__(self, ports, in_fifo, out_fifo):
        self.ports= ports
        self.in_fifo = in_fifo
        self.out_fifo = out_fifo

    def elaborate(self, platform):
        m = Module()

        isa_register = Memory(width=8, depth=8, init=[0, 0, 0, 0, 0, 0, 0, 0])
        isa_register_read = isa_register.read_port()
        isa_register_write = isa_register.write_port()

        addr = Signal(3)
        data = Signal(8)

        m.submodules.addr_buffer = addr_buffer = io.Buffer("i", self.ports.addr)
        m.submodules.data_buffer = data_buffer = io.Buffer("io", self.ports.data)
        m.submodules.cs_buffer = cs_buffer = io.Buffer("i", self.ports.cs)
        m.submodules.ior_buffer = ior_buffer = io.Buffer("i", self.ports.ior)
        m.submodules.iow_buffer = iow_buffer = io.Buffer("i", self.ports.iow)
        m.submodules.irq_buffer = irq_buffer = io.Buffer("i", self.ports.irq)
        m.submodules.irq2_buffer = irq2_buffer = io.Buffer("i", self.ports.irq2)

        m.d.comb += [
            addr.eq(addr_buffer.i),
        ]

        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += [
                    data_buffer.oe.eq(0),
                ]
                with m.If(cs_buffer.i & ~ior_buffer.i):
                    m.d.sync += [
                        self.in_fifo.w_data.eq(self.Command.READ_TRACE),
                        self.in_fifo.w_en.eq(1),
                    ]
                    #m.next = "READ"
                    m.next = "PREPARE"
                with m.Elif(cs_buffer.i & ~iow_buffer.i):
                    m.d.sync += [
                        self.in_fifo.w_data.eq(self.Command.WRITE_TRACE),
                        self.in_fifo.w_en.eq(1),
                    ]
                    #m.next = "WRITE"
                    m.next = "PREPARE"
                with m.Else():
                    m.d.sync += [
                        self.in_fifo.w_en.eq(0),
                    ]
                    m.next = "IDLE"

            with m.State("PREPARE"):
                m.d.sync += [
                    self.in_fifo.w_data.eq(addr),
                    self.in_fifo.w_en.eq(1),
                ]

                with m.If(~ior_buffer.i):
                    m.next = "READ"
                with m.Elif(~iow_buffer.i):
                    m.next = "WRITE"
            
            with m.State("READ"):
                m.d.sync += [
                    isa_register_read.addr.eq(addr),
                    data_buffer.oe.eq(0xff),
                    data_buffer.o.eq(isa_register_read.data),
                    # Debug TRACE
                    self.in_fifo.w_en.eq(1),
                    self.in_fifo.w_data.eq(isa_register_read.data)
                    #self.in_fifo.w_data.eq(0xAA)
                ]
                m.next = "CLEANUP"
            
            with m.State("WRITE"):
                m.d.sync += [
                    isa_register_write.addr.eq(addr),
                    isa_register_write.data.eq(data_buffer.i),
                    isa_register_write.en.eq(1),
                    # Debug TRACE
                    self.in_fifo.w_en.eq(1),
                    self.in_fifo.w_data.eq(data_buffer.i),
                ]
                m.next = "CLEANUP"

            with m.State("CLEANUP"):
                m.d.sync += [
                    self.in_fifo.w_en.eq(0),
                ]

                with m.If(ior_buffer.i & iow_buffer.i):
                    m.next = "IDLE"

        return m
    
class IsaInterface:
    def __init__(self, interface, logger):
        self._lower  = interface
        self._logger = logger
        self._level  = logging.DEBUG if self._logger.name == __name__ else logging.TRACE

    async def read(self):
        cmd = await self._lower.read()
        addr = await self._lower.read()
        data = await self._lower.read()

        self._logger.log(self._level, "ISA read: cmd=%s addr=%s data=%s", cmd, addr, data)

class IsaApplet(GlasgowApplet):
    logger = logging.getLogger(__name__)
    help = "Mini ISA interface (3bit address, 8bit data)"
    description = """
    Communicate with ISA host using stripped down version of the interface.
    """
    required_revision = "C0"

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        iface.add_subtarget(IsaSubtarget(
            ports = iface.get_port_group(
                addr = [
                    PinArgument(number = 0),     # A0
                    PinArgument(number = 1),     # A1
                    PinArgument(number = 2),     # A2
                ],
                data = [
                    PinArgument(number = 3),     # A3
                    PinArgument(number = 4),     # A4
                    PinArgument(number = 5),     # A5
                    PinArgument(number = 6),     # A6
                    PinArgument(number = 7),     # A7
                    PinArgument(number = 8),     # B0
                    PinArgument(number = 9),     # B1
                    PinArgument(number = 10),    # B2
                ],
                cs = PinArgument(number = 11),   # B3
                ior = PinArgument(number = 12),  # B4
                iow = PinArgument(number = 13),  # B5
                irq = PinArgument(number = 14),  # B6
                irq2 = PinArgument(number = 15), # B7
            ),
            in_fifo = iface.get_in_fifo(),
            out_fifo = iface.get_out_fifo(),
        ))

    async def run(self, device, args):
        iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)

        isa_internal_iface = IsaInterface(iface, self.logger)

        while True:
            await isa_internal_iface.read()

        return True
    
    @classmethod
    def tests(cls):
        from . import test
        return test.IsaAppletTestCase