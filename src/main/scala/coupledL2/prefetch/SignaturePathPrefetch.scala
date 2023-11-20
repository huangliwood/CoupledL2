// error when 000 signature occurs in pTable
package coupledL2.prefetch

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import coupledL2.HasCoupledL2Parameters
import xs.utils.CircularShift
import xs.utils.perf.HasPerfLogging
import xs.utils.sram.SRAMTemplate
import coupledL2.PfSource

case class SPPParameters(
  sTableEntries: Int = 256,
  sTagBits: Int = 12,
  pTableEntries: Int = 512,
  pTableDeltaEntries: Int = 4,
  pTableQueueEntries: Int = 4,
  lookCountBits: Int = 6,
  signatureBits: Int = 12,
  unpackQueueEntries: Int = 4,
  fTableEntries: Int = 32
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 32
}

trait HasSPPParams extends HasPrefetchParameters {
//  val p: Parameters
//  val cacheParams = p(L2ParamKey)
  val sppParams = SPPParameters()
  val sTagBits = sppParams.sTagBits
  val sTableEntries = sppParams.sTableEntries
  val pTableEntries = sppParams.pTableEntries
  val pTableWays = 1
  val pTableDeltaEntries = sppParams.pTableDeltaEntries
  val signatureBits = sppParams.signatureBits
  val pTableQueueEntries = sppParams.pTableQueueEntries
  val lookCountBits = sppParams.lookCountBits
  val unpackQueueEntries = sppParams.unpackQueueEntries
  val fTableEntries = sppParams.fTableEntries

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits
  val pTagBits = signatureBits - log2Up(pTableEntries)
  val fTagBits = fullAddressBits - offsetBits - log2Up(fTableEntries)

  val ENABLE_BP = true
  val ENABLE_NL = false

  def strideMap(a: SInt) : UInt = {
    val out = WireInit(0.U(3.W))
    when(a <= -5.S) {
      out := "b100".U
    } .elsewhen(a >= 5.S) {
      out := "b011".U
    } .elsewhen(a <= -3.S && a >= -4.S) {
      out := "b101".U
    } .elsewhen(a <= 4.S && a >= 3.S) {
      out := "b000".U
    } .otherwise {
      out := a.asUInt(2, 0)
    }
    out
  }

  def strideMap4dma(a: SInt): UInt = {
    val out = WireInit(0.U(5.W))
    when(a < -16.S) {
      out := (0.U)(5, 0)
    }.elsewhen(-16.S <= a && a < 16.S) {
      out := (a + 16.S)(5, 0)
    }.otherwise {
      out := 31.U
    }
    out
  }

  def makeSign(old_sig:UInt,new_delta:SInt)=(old_sig << 3) ^ new_delta.asUInt
  def makeSign(old_sig:UInt,new_delta:UInt)=(old_sig << 3) ^ new_delta.asUInt
}


abstract class SPPBundle(implicit val p: Parameters) extends Bundle with HasSPPParams
abstract class SPPModule(implicit val p: Parameters) extends Module with HasSPPParams with HasPerfLogging 

class SignatureTableReq(implicit p: Parameters) extends SPPBundle {
  val pageAddr = UInt(pageAddrBits.W)
  val blkOffset = UInt(blkOffsetBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val isBP = Bool()
  def get_blkAddr = Cat(pageAddr,blkOffset)
  def get_accessAddr = Cat(pageAddr,blkOffset,0.U(offsetBits.W))
}
class BreakPointReq(implicit p: Parameters) extends SPPBundle{
  val pageAddr = UInt(pageAddrBits.W)
  val parent_sig = Vec(1,UInt(signatureBits.W))
  val offset = UInt(blkOffsetBits.W)
}

class SignatureTableResp(implicit p: Parameters) extends SPPBundle {
  val signature = UInt(signatureBits.W)
  val delta = SInt((blkOffsetBits + 1).W)
  val block = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val isBP = Bool()
}

class PatternTableResp(implicit p: Parameters) extends SPPBundle {
  val deltas = Vec(pTableDeltaEntries, SInt((blkOffsetBits + 1).W))
  val block = UInt((pageAddrBits + blkOffsetBits).W)
  val degree = UInt(lookCountBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
}

class UnpackResp(implicit p: Parameters) extends SPPBundle {
  val prefetchBlock = UInt((pageAddrBits + blkOffsetBits).W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val degree = UInt(lookCountBits.W)
}

class DeltaEntry(implicit p: Parameters) extends SPPBundle {
  val delta = SInt((blkOffsetBits + 1).W)
  val cDelta = UInt(4.W)

  def apply(delta: SInt, cDelta: UInt) = {
    val entry = WireInit(0.U.asTypeOf(this))
    entry.delta := delta
    entry.cDelta := cDelta
    entry
  }
}

class SignatureTable(parentName: String = "Unknown")(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableReq))
    val resp = DecoupledIO(new SignatureTableResp)
    val bp_update = Flipped(ValidIO(new BreakPointReq))
  })
  assert(pageAddrBits>=(2 * log2Up(sTableEntries)),s"pageAddrBits ${pageAddrBits} ,it as least ${2 * log2Up(sTableEntries)} bits to use hash")
  def hash1(addr:    UInt) = addr(log2Up(sTableEntries) - 1, 0)
  def hash2(addr:    UInt) = addr(2 * log2Up(sTableEntries) - 1, log2Up(sTableEntries))
  def idx(addr:      UInt) = hash1(addr) ^ hash2(addr)
  def tag(addr:      UInt) = addr(sTagBits + log2Up(sTableEntries) - 1, log2Up(sTableEntries))
  def sTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(sTagBits.W)
    val signature = UInt(signatureBits.W)
    val lastBlockOff = UInt(blkOffsetBits.W)
  }

  val sTable = Module(
    new SRAMTemplate(sTableEntry(), set = sTableEntries, way = 1, 
      bypassWrite = true, 
      shouldReset = true, 
      hasMbist = cacheParams.hasMbist, 
      hasShareBus = cacheParams.hasShareBus,
      hasClkGate = enableClockGate, 
      parentName = parentName
    ))
  // --------------------------------------------------------------------------------
  // stage 0
  // --------------------------------------------------------------------------------
  // read sTable
  val s0_valid = io.req.valid
  val s0_req = io.req.bits
  sTable.io.r.req.valid       := s0_valid
  sTable.io.r.req.bits.setIdx := idx(s0_req.pageAddr)
  // --------------------------------------------------------------------------------
  // stage 1
  // --------------------------------------------------------------------------------
  // get sTable read data, because SRAM read delay, should send to S2 to handle rdata
  // request bp to PT if needed
  val s1_valid = RegNext(s0_valid,false.B)
  val s1_req = RegNext(s0_req,0.U.asTypeOf(s0_req.cloneType))
  val s1_hit = s1_valid && sTable.io.r.resp.data(0).tag === tag(s1_req.pageAddr)
  // --------------------------------------------------------------------------------
  // stage 2
  // --------------------------------------------------------------------------------
  // update sTable & req pTable
  val s2_valid = RegNext(s1_valid,false.B)
  val s2_hit = RegNext(s1_hit,false.B)
  val s2_req = RegEnable(s0_req,s1_valid)
  val s2_accessedPage  = s2_req.pageAddr
  val s2_accessedBlockOff = s2_req.blkOffset

  val s2_isBP = s2_req.isBP
  val s2_entryData = RegEnable(sTable.io.r.resp.data(0), 0.U.asTypeOf(sTableEntry()),  s1_valid)

  val s2_oldSignature = Mux(s2_hit, s2_entryData.signature, 0.U)
  val s2_oldBlockOff =  s2_entryData.lastBlockOff
  // used for prefetch hit traning and probe one delta
  val s2_probeDelta = Mux(s2_isBP,s2_oldSignature.head(9).tail(6).asSInt,0.S)

  val s2_newDelta     = Mux(s2_hit && !s2_isBP, s2_accessedBlockOff.asSInt - s2_entryData.lastBlockOff.asSInt, 0.S) // should hold 0 when miss

  val bp_hit = WireInit(false.B)
  val bp_prePredicted_blkOff = WireInit(0.U(blkOffsetBits.W))
  val bp_matched_sig = WireInit(0.U(signatureBits.W))

  //bp, reg realize
  def breakPointEntry() = new Bundle() {
    val valid = Bool()
    val tag = UInt(pageAddrBits.W)
    val parent_sig = Vec(1, UInt(signatureBits.W))
    val prePredicted_blkOffset = UInt(blkOffsetBits.W)
  }

  // write
  val bpTable = RegInit(VecInit(Seq.fill(256)(0.U.asTypeOf(breakPointEntry()))))
  val bp_page = io.bp_update.bits.pageAddr
  bpTable(idx(bp_page)).valid := io.bp_update.valid
  bpTable(idx(bp_page)).tag := bp_page
  bpTable(idx(bp_page)).parent_sig.zip(io.bp_update.bits.parent_sig).foreach(x => x._1 := x._2)
  bpTable(idx(bp_page)).prePredicted_blkOffset := io.bp_update.bits.offset

  // bp read
  val s2_bp_access_index = idx(s2_accessedPage)(4, 0)
  val rotate_sig = VecInit(Seq.fill(4)(0.U(signatureBits.W)));
  dontTouch(rotate_sig)
  for (i <- 0 until (4)) {
    rotate_sig(i) := CircularShift(bpTable(s2_bp_access_index).parent_sig.head).left(3 * i)
  }
 
  bp_hit := ENABLE_BP.asBool && s2_valid && bpTable(s2_bp_access_index).tag === s2_accessedPage && rotate_sig.map(_ === s2_oldSignature).reduce(_ || _)
  bp_prePredicted_blkOff := bpTable(s2_bp_access_index).prePredicted_blkOffset
  val bp_matched_index = WireInit(0.U(2.W))
  bp_matched_sig := rotate_sig(bp_matched_index)

  sTable.io.w.req.valid := s2_valid && !s2_req.isBP
  sTable.io.w.req.bits.setIdx := idx(s2_accessedPage)
  sTable.io.w.req.bits.data(0).valid := true.B
  sTable.io.w.req.bits.data(0).tag := tag(s2_accessedPage)
  sTable.io.w.req.bits.data(0).signature := makeSign(s2_oldSignature,s2_newDelta)//TODO: there should hold origin delta signal!!
  sTable.io.w.req.bits.data(0).lastBlockOff := s2_accessedBlockOff

  val resp_delta = Mux(s2_isBP, s2_probeDelta, s2_newDelta)
  val s2_accessedBlock = Mux(s2_isBP, Cat(s2_accessedPage,s2_oldBlockOff), s2_req.get_blkAddr)
  io.resp.valid := resp_delta =/= 0.S && s2_hit && s2_valid
  io.resp.bits.delta  :=resp_delta
  io.resp.bits.source := s2_req.source
  io.resp.bits.needT := s2_req.needT
  io.resp.bits.isBP := s2_isBP
  when(bp_hit){
    io.resp.bits.signature := bp_matched_sig
    io.resp.bits.block := (s2_accessedBlock >> blkOffsetBits << blkOffsetBits) + bp_prePredicted_blkOff
  }.otherwise {
    io.resp.bits.signature := s2_oldSignature
    io.resp.bits.block := s2_accessedBlock
  }

  io.req.ready := sTable.io.r.req.ready

  XSPerfAccumulate("spp_st_req_nums",io.resp.valid)
  XSPerfAccumulate("spp_st_reqfire_nums",io.resp.fire)
  XSPerfAccumulate("spp_st_bp_req", bp_hit)
  XSPerfAccumulate("spp_st_bp_update",io.bp_update.valid)
}

class PatternTable(parentName:String="Unkown")(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new SignatureTableResp))
    val resp = DecoupledIO(new PatternTableResp)
    val from_ghr = (new Bundle {
        val deadCov_state = Input(UInt(PfcovState.bits.W))
        val hitAcc_state = Input(UInt(PfaccState.bits.W))
        val global_queue_used = Input((UInt(6.W)))
    })
    val pt2st_bp = ValidIO(new BreakPointReq)
  })

  def idx(addr:      UInt) = addr(log2Up(pTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(signatureBits - 1, log2Up(pTableEntries))
  def pTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(pTagBits.W)
    val deltaEntries = Vec(pTableDeltaEntries, new DeltaEntry())
    val count = UInt(4.W)
  }

  val pTable = Module(
    new SRAMTemplate(pTableEntry(), set = pTableEntries, way = 1, 
      bypassWrite = true, 
      shouldReset = true, 
      hasMbist = cacheParams.hasMbist, 
      hasShareBus = cacheParams.hasShareBus,
      hasClkGate = enableClockGate, 
      parentName = parentName
    ))

  val q = Module(new Queue(chiselTypeOf(io.req.bits), pTableQueueEntries, flow = true, pipe = false))
  q.io.enq <> io.req
  val issueReq = RegInit(0.U.asTypeOf(new SignatureTableResp))
  when(q.io.deq.valid){
    issueReq := q.io.deq.bits
  }
  val s_idle :: s_lookahead0 :: s_updateTable :: s_lookahead :: Nil = Enum(4)
  val state = RegInit(s_idle)

  val enprefetch = WireInit(false.B)
  val enprefetchnl = WireInit(false.B)
  val lookCount = RegInit(0.U(lookCountBits.W))
  val miniCount = lookCount >> 2.U
  //read pTable
  // --------------------------------------------------------------------------------
  // stage 0
  // -------------------------------------------------------------------------------
  // 1. just read data from sram and check for hit
  // 2. bp update operation
  val s1_continue = WireInit(false.B)
  val s1_first_flag = RegInit(false.B)
  val s1_valid = WireInit(false.B)
  val s0_valid = WireInit((state === s_lookahead0 || state === s_lookahead) && ~s1_valid)
  val s0_readResult = WireInit(0.U.asTypeOf(pTableEntry()))
  val s0_lastSignature = WireInit(0x1ff.U(signatureBits.W))
  val s0_current = WireInit(0.U.asTypeOf(new SignatureTableResp));dontTouch(s0_current)
  //| signature | delta | block |

  pTable.io.r.req.valid := q.io.deq.fire || state === s_updateTable || s1_continue
  pTable.io.r.req.bits.setIdx := idx(s0_current.signature)
  s0_readResult := pTable.io.r.resp.data(0)

  val enbp = WireInit(false.B)
  val bp_update = WireInit(false.B)
  io.pt2st_bp.valid := enbp && bp_update
  io.pt2st_bp.bits.pageAddr := s0_current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)
  io.pt2st_bp.bits.parent_sig(0) := s0_lastSignature
  io.pt2st_bp.bits.offset := s0_current.block(blkOffsetBits - 1, 0)
  q.io.deq.ready := ~s0_valid

  // --------------------------------------------------------------------------------
  // stage 1
  // -------------------------------------------------------------------------------
  //s1 calculate value for next update
  s1_valid := RegNext(s0_valid,false.B)
  val s1_current = RegEnable(s0_current,s0_valid)
  val s1_readResult = RegEnable(s0_readResult,s0_valid)
  val s1_lastDelta = WireInit(s1_current.delta)
  val s1_wdeltaEntries = WireInit(VecInit(Seq.fill(pTableDeltaEntries)(0.U.asTypeOf(new DeltaEntry()))))
  val s1_maxEntry = s1_readResult.deltaEntries.reduce((a, b) => Mux(a.cDelta >= b.cDelta, a, b))
  //set output
  val s1_delta_list = s1_readResult.deltaEntries.map(x => Mux(x.cDelta > miniCount.asUInt, x.delta, 0.S))
  val s1_delta_list_nl = s1_delta_list.map(_ => 1.S((blkOffsetBits + 1).W))
//  val s1_delta_list = s1_delta_list.map(x => RegEnable(x,s1_valid))
  val s1_delta_list_checked = s1_delta_list.map(x =>
    Mux((s1_current.block.asSInt + x).asUInt(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === s1_current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits),
      x, 0.S))
  val s1_hit = WireInit(s1_readResult.valid && tag(s1_current.signature) === s1_readResult.tag)

  when(state === s_lookahead0){
    s1_first_flag := true.B
  }.elsewhen(state === s_lookahead){
    s1_first_flag := false.B
  }

  s1_continue := s1_valid && s1_hit && state === s_lookahead

  val s1_count = WireInit(0.U(4.W));dontTouch(s1_count)
  val s1_exist = s1_readResult.deltaEntries.map(_.delta === s1_lastDelta).reduce(_ || _)
  val s1_temp = s1_readResult.deltaEntries.map(x => Mux(x.delta === s1_lastDelta, (new DeltaEntry).apply(s1_lastDelta, x.cDelta + 1.U), x))
  val smallest: SInt = s1_readResult.deltaEntries.reduce((a, b) => Mux(a.cDelta < b.cDelta, a, b)).delta
  val indexToReplace: UInt = s1_readResult.deltaEntries.indexWhere(a => a.delta === smallest)
  //predict
  val issued = s1_delta_list_checked.map(a => Mux(a =/= 0.S, 1.U, 0.U)).reduce(_ +& _)
  val testOffset =Mux(issued =/= 0.U,(s1_current.block.asSInt + s1_maxEntry.delta).asUInt,s1_current.block)
  // val testOffset = (current.block.asSInt + maxEntry.delta).asUInt
  //same page?
  val samePage = (testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === s1_current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits))
  enprefetch := s1_valid && s1_hit && issued =/= 0.U
  //forward hold dequeue data
  when(state === s_idle){
    s0_current.signature := q.io.deq.bits.signature
  }.elsewhen(state === s_lookahead0){
    s0_current.signature := issueReq.signature
  }.otherwise{
    s0_current.signature := makeSign(s1_current.signature,strideMap(s1_maxEntry.delta))
  }
  when(state === s_lookahead){
    s0_current.delta := s1_maxEntry.delta
    s0_current.block := testOffset //TODO: need opimize?
    s0_current.isBP := false.B
  }.otherwise{
    s0_current.delta := issueReq.delta
    s0_current.block := issueReq.block
    s0_current.isBP := issueReq.isBP
  }

  // write
  when(s1_valid && s1_hit) {
    when(s1_exist) {
      //counter overflow
      when(s1_readResult.count + 1.U === ((1.U << s1_count.getWidth).asUInt - 1.U)) {
        s1_wdeltaEntries := s1_temp.map(x => (new DeltaEntry).apply(x.delta, x.cDelta >> 1.asUInt))
      } .otherwise {
        s1_wdeltaEntries := s1_temp
      }
    } .otherwise {
      //to do replacement
      s1_wdeltaEntries := VecInit.tabulate(s1_readResult.deltaEntries.length) { i =>
        Mux((i.U === indexToReplace), (new DeltaEntry).apply(s1_lastDelta, 1.U), s1_readResult.deltaEntries(i))
      }
    }
    s1_count := s1_wdeltaEntries.map(_.cDelta).reduce(_ + _) //todo: must be optimized!
    //to consider saturate here
  } .otherwise {
    s1_wdeltaEntries(0).delta := s1_lastDelta
    s1_wdeltaEntries(0).cDelta := 1.U
    s1_count := 1.U
  }

  //write pTable
  val enwrite = s1_valid && state === s_lookahead0
  pTable.io.w.req.valid := RegNext(enwrite,false.B) //we only modify-write on demand requests
  pTable.io.w.req.bits.setIdx := RegEnable(idx(s1_current.signature),enwrite)
  pTable.io.w.req.bits.data(0).valid := true.B
  pTable.io.w.req.bits.data(0).deltaEntries := RegEnable(s1_wdeltaEntries,enwrite)
  pTable.io.w.req.bits.data(0).count := RegEnable(s1_count,enwrite)
  pTable.io.w.req.bits.data(0).tag := RegEnable(tag(s1_current.signature),enwrite)

  io.resp.valid := enprefetch || enprefetchnl
  io.resp.bits.block := s1_current.block
  io.resp.bits.deltas := s1_delta_list_checked
  io.resp.bits.degree := lookCount
  io.resp.bits.source := s1_current.source
  io.resp.bits.needT := s1_current.needT

  when(enprefetchnl) {
    io.resp.bits.deltas := s1_delta_list_nl
  }
  //FSM
  switch(state) {
    is(s_idle) {
      when(q.io.deq.fire) {
        state := s_lookahead0
      }
    }
    is(s_lookahead0) {
      when(s1_valid && !s1_current.isBP) {
        state := s_updateTable
      }.elsewhen(s1_valid){
        state := s_lookahead
      }
    }
    is(s_updateTable) {
      state := s_lookahead
    }
    is(s_lookahead) {
      when(s1_valid){
        when(s1_continue || s1_first_flag) {
          when(issued =/= 0.U) {
            when(samePage  && (s1_maxEntry.cDelta > miniCount)) {
              lookCount := lookCount + 1.U
            } .otherwise {
              lookCount := 0.U
              state := s_idle
            }
          }.otherwise {
            when(lookCount>=4.U){
              bp_update := true.B
            }
            lookCount := 0.U
            state := s_idle
          }
        } .otherwise {
          when(lookCount <= 1.U) {
            val testOffset = s1_current.block + 1.U
            when(testOffset(pageAddrBits + blkOffsetBits - 1, blkOffsetBits) === s1_current.block(pageAddrBits + blkOffsetBits - 1, blkOffsetBits)) {
              enprefetchnl := false.B
            }
          }
          lookCount := 0.U
          state := s_idle
        }
      }
    }
  }

  //perf
  XSPerfAccumulate("spp_pt_bp_nums",io.pt2st_bp.valid)
  XSPerfAccumulate("spp_pt_lookahead2",state === s_lookahead && s1_valid && s1_continue)
  XSPerfAccumulate("spp_pt_nextLine",state === s_lookahead && enprefetchnl)
  XSPerfAccumulate("spp_pt_cross_page",state === s_lookahead && s1_valid && samePage)
//  XSPerfAccumulate(s"spp_pt_do_nextline", enprefetchnl)
//  for (i <- 0 until pTableEntries) {
//    XSPerfAccumulate(s"spp_pt_touched_entry_onlyset_${i.toString}", pTable.io.r.req.bits.setIdx === i.U(log2Up(pTableEntries).W)
//    )
//  }
}

//Can add eviction notify or cycle counter for each entry
class Unpack(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new PatternTableResp))
    val resp = ValidIO(new UnpackResp)
  })

  def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(fullAddressBits - offsetBits - 1, log2Up(fTableEntries))

  def fTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(fTagBits.W)
  }
  val fTable = RegInit(VecInit(Seq.fill(fTableEntries)(0.U.asTypeOf(fTableEntry()))))

  val inProcess = RegInit(false.B)
  val endeq = WireInit(false.B)

  val q = Module(new Queue(chiselTypeOf(io.req.bits), unpackQueueEntries,flow = true, pipe = false))
  q.io.enq <> io.req //change logic to replace the tail entry
  val req= RegInit(0.U.asTypeOf(new PatternTableResp))
  when(q.io.deq.fire){
    req := q.io.deq.bits
  }

  val req_deltas = RegInit(VecInit(Seq.fill(pTableDeltaEntries)(0.S((blkOffsetBits + 1).W))))
  val issue_finish = req_deltas.map(_ === 0.S).reduce(_ && _)
  q.io.deq.ready := !inProcess || issue_finish || endeq
  when(q.io.deq.fire) {
    req_deltas := q.io.deq.bits.deltas
  }

  val enresp = WireInit(false.B)
  val extract_delta = req_deltas.reduce((a, b) => Mux(a =/= 0.S, a, b))
  val prefetchBlock = (req.block.asSInt + extract_delta).asUInt

  val hit = WireInit(false.B)
  val readResult = WireInit(0.U.asTypeOf(fTableEntry()))
  readResult := fTable(idx(prefetchBlock))
  hit := readResult.valid && tag(prefetchBlock) === readResult.tag

  when(enresp && !hit) {
    fTable(idx(prefetchBlock)).valid := true.B
    fTable(idx(prefetchBlock)).tag := tag(prefetchBlock)
  }

  io.resp.valid := enresp && !hit
  io.resp.bits.prefetchBlock := prefetchBlock
  io.resp.bits.degree := req.degree
  io.resp.bits.source := req.source
  io.resp.bits.needT := req.needT

  when(inProcess) {
    when(!issue_finish) {
      val cnt: UInt = req_deltas.count(_ =/= 0.S)
      enresp := true.B
      // req_deltas := req_deltas.map(a => Mux(a === extract_delta, 0.S, a))
      when(cnt === 1.U) {
        endeq := true.B
        when(!q.io.deq.fire) {
          req_deltas := req_deltas.map(a => Mux(a === extract_delta, 0.S, a))
        }
      } .otherwise {
        req_deltas := req_deltas.map(a => Mux(a === extract_delta, 0.S, a))
      }
    } .otherwise {
      when(!q.io.deq.fire) {
        inProcess := false.B
      }
    }
  } .otherwise {
    when(q.io.deq.fire) {
      inProcess := true.B
    }
  }
 XSPerfAccumulate("spp_filter_recv", io.req.valid)
 XSPerfAccumulate("spp_filter_nums", q.io.deq.fire && hit)
 XSPerfAccumulate("spp_filter_req", io.resp.valid)
//  for (off <- (-63 until 64 by 1).toList) {
//    if (off < 0) {
//      XSPerfAccumulate("spp_pt_pfDelta_neg_" + (-off).toString, hit && extract_delta === off.S((blkOffsetBits + 1).W))
//    } else {
//      XSPerfAccumulate("spp_pt_pfDelta_pos_" + off.toString, hit && extract_delta === off.S((blkOffsetBits + 1).W))
//    }
//  }
}

class SignaturePathPrefetch(parentName:String="Unkown")(implicit p: Parameters) extends SPPModule {
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain)) //train from mshr ,now recommand using MISS
    val req = DecoupledIO(new PrefetchReq) //issue to current or next-level cache
    val hint_req = DecoupledIO(new PrefetchReq)
    val resp = Flipped(DecoupledIO(new PrefetchResp)) //fill request from the next-level cache, using this to update filter
    val from_ghr = Flipped(ValidIO(new Bundle {
        val deadCov_state = UInt(PfcovState.bits.W)
        val hitAcc_state = UInt(PfaccState.bits.W)
        val global_queue_used = (UInt(6.W))
    }))
  })
  println(s"pageAddrBits: ${pageAddrBits}")
  println(s"log2Up(sTableEntries): ${log2Up(sTableEntries)}")
  println(s"fullAddressBits: ${fullAddressBits}")
  println(s"pageOffsetBits: ${pageOffsetBits}")
  println(s"sTagBits: ${sTagBits}")
  println(s"stableEntries: ${sTableEntries}")
  println(s"ptableEntries: ${pTableEntries}")
  println(s"pTagBits: ${pTagBits}")

  val sTable = Module(new SignatureTable(parentName + "stable_"))
  val pTable = Module(new PatternTable(parentName + "ptable_"))
  val unpack = Module(new Unpack)

  val oldAddr = io.train.bits.addr //received request from L1 cache
  val pageAddr = getPPN(oldAddr)
  val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

  // might be lack of prefetch requests
  io.train.ready := sTable.io.req.ready

  sTable.io.req.valid := io.train.valid // already filtered
  sTable.io.req.bits.pageAddr := pageAddr
  sTable.io.req.bits.blkOffset := blkOffset
  sTable.io.req.bits.needT := io.train.bits.needT
  sTable.io.req.bits.source := io.train.bits.source
  sTable.io.req.bits.isBP := io.train.bits.state === AccessState.PREFETCH_HIT

  pTable.io.req <> sTable.io.resp //to detail
  pTable.io.pt2st_bp := DontCare
  pTable.io.pt2st_bp <> sTable.io.bp_update
  pTable.io.resp <> unpack.io.req

  val newAddr = Cat(unpack.io.resp.bits.prefetchBlock, 0.U(offsetBits.W))
  val ghr_deadCov_state = RegEnable(io.from_ghr.bits.deadCov_state, 1.U, io.from_ghr.valid)
  val ghr_hitAcc_state = RegEnable(io.from_ghr.bits.hitAcc_state, 1.U, io.from_ghr.valid)
  val send2Llc = WireInit(false.B)
  send2Llc := unpack.io.resp.bits.degree > 1.U && (io.from_ghr.bits.global_queue_used >= 24.U || ghr_deadCov_state > PfcovState.p_25)

  pTable.io.from_ghr.deadCov_state := ghr_deadCov_state
  pTable.io.from_ghr.hitAcc_state := ghr_hitAcc_state
  pTable.io.from_ghr.global_queue_used := io.from_ghr.bits.global_queue_used

  io.req.valid := unpack.io.resp.valid
  io.req.bits.tag := parseFullAddress(newAddr)._1
  io.req.bits.set := parseFullAddress(newAddr)._2
  io.req.bits.needT := unpack.io.resp.bits.needT
  io.req.bits.source := unpack.io.resp.bits.source
  io.req.bits.pfVec := PfSource.SPP
  //hint req 
  io.hint_req.valid := unpack.io.resp.valid && send2Llc
  io.hint_req.bits := io.req.bits 


  io.resp.ready := true.B
  //perf
  XSPerfAccumulate( "spp_recv_train", io.train.fire)
  XSPerfAccumulate( "spp_recv_st", sTable.io.resp.fire)
  XSPerfAccumulate( "spp_recv_pt", Mux(pTable.io.resp.fire, pTable.io.resp.bits.deltas.map(a => Mux(a =/= 0.S, 1.U, 0.U)).reduce(_ +& _), 0.U))
  XSPerfAccumulate( "spp_hintReq", io.hint_req.valid)
}