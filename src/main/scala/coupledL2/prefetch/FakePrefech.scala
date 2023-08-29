package coupledL2.prefetch
import chisel3._
import chisel3.util._
import chisel3.experimental._
import chisel3.util.{DecoupledIO, HasBlackBoxResource}
import chipsalliance.rocketchip.config.Parameters
import chisel3.util.experimental.BoringUtils
import coupledL2.utils.XSPerfAccumulate

class PerfInfoIO extends Bundle {
  val XSPERF_CLEAN = Input(Bool())
  val XSPERF_DUMP = Input(Bool())
}

case class FakePrefetchPrarameters() extends PrefetchParameters{
  override val hasPrefetchBit: Boolean = true
  override val inflightEntries: Int = 16
  val prefetchName="Fake Prefetcher for blackBox"
}
class FakePrefetch(implicit p:Parameters) extends BlackBox with HasBlackBoxResource{
  val io = IO(new Bundle() {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val pf = new PrefetchIO()
    val perf = new PerfInfoIO()
  })

  BoringUtils.addSink(io.perf.XSPERF_CLEAN, "XSPERF_CLEAN")
  BoringUtils.addSink(io.perf.XSPERF_DUMP, "XSPERF_DUMP")

  addResource("/FakePrefetch.v")
}
