package coupledL3

import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.tilelink.TLMessages._
import chisel3._
import chisel3.util._
import coupledL3.utils._
import utility._

class InputBuffer(flow: Boolean = true, entries: Int = 4)(implicit p: Parameters) extends L3Module with noninclusive.HasClientInfo {

    val io = IO(new Bundle() {
      val in          = Flipped(DecoupledIO(new TaskBundle))
      val out         = DecoupledIO(new TaskBundle)

      val sinkEntrance = Flipped(ValidIO(new L3Bundle {
        val tag = UInt(tagBits.W)
        val set = UInt(setBits.W)
      }))

      val mshrStatus  = Vec(mshrsAll, Flipped(ValidIO(new MSHRBlockAInfo)))
    })

    val buffer = Module(new Queue(new TaskBundle, entries, flow = flow))
    buffer.io.enq <> io.in
    
    val taskArbEntries = entries / 2
    val taskArbBuffer = RegInit(VecInit(Seq.fill(taskArbEntries)(0.U.asTypeOf(Valid(new TaskBundle)))))
    val taskArbBufferValidVec = VecInit(taskArbBuffer.map(_.valid)).asUInt
    val taskArb = Module(new FastArbiter(new TaskBundle, taskArbEntries))


    // --------------------------------------------------------------------------
    //  Insert into taskArb, output a chosen task when specific condition is achieved
    // --------------------------------------------------------------------------
    val insertIdx = PriorityEncoder(~taskArbBufferValidVec)
    buffer.io.deq.ready := !taskArbBuffer(insertIdx).valid
    when(buffer.io.deq.fire) {
      taskArbBuffer(insertIdx).valid := true.B
      taskArbBuffer(insertIdx).bits := buffer.io.deq.bits
    }

    val bufferTimer = RegInit(VecInit(Seq.fill(taskArbEntries)(0.U(16.W))))
    taskArbBuffer zip bufferTimer map {
      case (e, t) =>
        when(e.valid) { t := t + 1.U }
        when(RegNext(RegNext(e.valid) && !e.valid)) { t := 0.U }
        assert(t < 10000.U, "ReqBuf Leak set:0x%x tag:0x%x addr:0x%x source:%d opcode:%d param:%d", e.bits.set, e.bits.set, Cat(e.bits.tag, e.bits.set, Fill(6, 0.U)), e.bits.sourceId, e.bits.opcode, e.bits.param)

        val enable = RegNext(e.valid) && !e.valid
        XSPerfHistogram(cacheParams, "reqBuf_timer", t, enable, 0, 20, 1, right_strict = true)
        XSPerfHistogram(cacheParams, "reqBuf_timer", t, enable, 20, 400, 20, left_strict = true)
        XSPerfMax(cacheParams, "max_reqBuf_timer", t, enable)
    }

    
    def sameSet (a: TaskBundle, b: TaskBundle):     Bool = a.set === b.set
    def sameSet (a: TaskBundle, b: MSHRBlockAInfo): Bool = a.set === b.set
    def sameClientSet (a: TaskBundle, b: TaskBundle):     Bool = a.set(clientSetBits-1, 0) === b.set(clientSetBits-1, 0)
    def sameClientSet (a: TaskBundle, b: MSHRBlockAInfo): Bool = a.set(clientSetBits-1, 0) === b.set(clientSetBits-1, 0)

    // count ways
    def countWaysOH(cond: (MSHRBlockAInfo => Bool)): UInt = {
      VecInit(io.mshrStatus.map(s =>
        Mux(
          s.valid && cond(s.bits),
          UIntToOH(s.bits.way, cacheParams.ways),
          0.U(cacheParams.ways.W)
        )
      )).reduceTree(_ | _)
    }
    def occWays(a: TaskBundle): UInt = countWaysOH(s => !s.willFree && sameSet(a, s))

    // count client ways
    def countClientWaysOH(cond: (MSHRBlockAInfo => Bool)): UInt = {
      VecInit(io.mshrStatus.map(s =>
        Mux(
          s.valid && cond(s.bits),
          UIntToOH(s.bits.clientWay, clientWays),
          0.U(clientWays.W)
        )
      )).reduceTree(_ | _)
    }
    def occClientWays(a: TaskBundle): UInt = countClientWaysOH(s => !s.willFree && sameClientSet(a, s))

    def noFreeWay(a: TaskBundle): Bool = !Cat(~occWays(a)).orR
    def noFreeClientWay(a: TaskBundle): Bool = !Cat(~occClientWays(a)).orR

    def addrConflict(a: TaskBundle, s: MSHRBlockAInfo): Bool = {
      // a.set === s.set && (a.tag === s.reqTag || a.tag === s.metaTag && s.needRelease) // TODO: reduce set blocking for L3 ?
      a.set === s.set && (a.tag === s.reqTag || a.tag === s.metaTag) // TODO: reduce set blocking for L3 ?
    }
    def conflictMask(a: TaskBundle): UInt = VecInit(io.mshrStatus.map(s =>
    s.valid && addrConflict(a, s.bits) && !s.bits.willFree)).asUInt
    def conflict(a: TaskBundle): Bool = conflictMask(a).orR

    val hasEmptyWay = Cat(io.out.bits.wayMask).orR
    val hasEmptyClientWay = Cat(io.out.bits.clientWayMask).orR
    dontTouch(hasEmptyWay)
    dontTouch(hasEmptyClientWay)
    assert(!(io.out.fire && !hasEmptyWay), "[InputBuffer] not enough wayMask set:%x tag:%x", io.out.bits.set, io.out.bits.tag)
    assert(!(io.out.fire && !hasEmptyClientWay), "[InputBuffer] not enough clientWayMask set:%x tag:%x", io.out.bits.set, io.out.bits.tag)

    // --------------------------------------------------------------------------
    // Output
    // --------------------------------------------------------------------------
    taskArb.io.in.zip(taskArbBuffer).foreach{
      case(in, buffer) =>
        val noFreeWay_arb = noFreeWay(buffer.bits)
        val noFreeClientWay_arb = noFreeClientWay(buffer.bits)
        val conflict_arb = conflict(buffer.bits)
        val sinkEntranceBlock_arb = io.sinkEntrance.valid && io.sinkEntrance.bits.set === buffer.bits.set
        val readyToOutput_arb = !noFreeWay_arb && !noFreeClientWay_arb && !conflict_arb //&& !sinkEntranceBlock_arb
 
        in.valid := buffer.valid && readyToOutput_arb
        in.bits := buffer.bits
        in.bits.wayMask := ~occWays(in.bits)
        in.bits.clientWayMask := ~occClientWays(io.out.bits)

        when(in.fire) {
          buffer.valid := false.B
        }
    }

    io.out <> taskArb.io.out
}