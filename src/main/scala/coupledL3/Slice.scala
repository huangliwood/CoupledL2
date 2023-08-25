/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

package coupledL3

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.util.leftOR
import chipsalliance.rocketchip.config.Parameters
import coupledL3.utils._
import coupledL3.debug._
import coupledL3.noninclusive.ProbeHelper
import utility.RegNextN
import chisel3.util.experimental.BoringUtils

class Slice()(implicit p: Parameters) extends L3Module with DontCareInnerLogic {
  val io = IO(new Bundle {
    val in = Flipped(TLBundle(edgeIn.bundle))
    val out = TLBundle(edgeOut.bundle)
    val msStatus = topDownOpt.map(_ => Vec(mshrsAll, ValidIO(new MSHRStatus)))
    val dirResult = topDownOpt.map(_ => ValidIO(new DirResult))
  })

  val reqArb = Module(new RequestArb())
  val a_reqBuf = Module(new RequestBuffer_1)
  val mainPipe = Module(new MainPipe())
  val mshrCtl = Module(new MSHRCtl())
  val directory = Module(new Directory())
  val dataStorage = Module(new DataStorage())
  val refillUnit = Module(new RefillUnit())
  val sinkA = Module(new SinkA)
  val sinkC = Module(new SinkC)
  val sourceC = Module(new SourceC)
  val grantBuf = if (!useFIFOGrantBuffer) Module(new GrantBuffer) else Module(new GrantBufferFIFO)
  val refillBuf = Module(new MSHRBuffer(wPorts = 2))
  val releaseBuf = Module(new MSHRBuffer(wPorts = 3))
  val putDataBuf = Module(new LookupBuffer(entries = lookupBufEntries))

  val probeHelper = Module(new ProbeHelper(entries = 5, enqDelay = 1))
  val clientDirectory = Module(new noninclusive.ClientDirectory())

  reqArb.io.fromProbeHelper.blockSinkA := false.B

  // We will get client directory result after 2 cyels of delay
  probeHelper.io.clientDirResult.valid := RegNextN(reqArb.io.dirRead_s1.valid, 2, Some(false.B)) // TODO: Optimize for clock gate
  probeHelper.io.clientDirResult.bits := clientDirectory.io.resp


  clientDirectory.io.read <> reqArb.io.clientDirRead_s1
  clientDirectory.io.tagWReq <> mainPipe.io.clientTagWReq
  clientDirectory.io.metaWReq <> mainPipe.io.clientMetaWReq


  a_reqBuf.io.in <> sinkA.io.toReqArb
  a_reqBuf.io.mshrStatus := mshrCtl.io.toReqBuf
  a_reqBuf.io.mainPipeBlock := mainPipe.io.toReqBuf
  a_reqBuf.io.sinkEntrance := reqArb.io.sinkEntrance
  a_reqBuf.io.pipeFlow_s1 := reqArb.io.pipeFlow_s1
  a_reqBuf.io.pipeFlow_s2 := mainPipe.io.pipeFlow_s2
  a_reqBuf.io.pipeFlow_s3 := mainPipe.io.pipeFlow_s3

  reqArb.io.sinkA <> a_reqBuf.io.out
  reqArb.io.ATag := a_reqBuf.io.ATag
  reqArb.io.ASet := a_reqBuf.io.ASet


  reqArb.io.sinkC <> sinkC.io.toReqArb
  reqArb.io.dirRead_s1 <> directory.io.read
  reqArb.io.taskToPipe_s2 <> mainPipe.io.taskFromArb_s2
  reqArb.io.mshrTask <> mshrCtl.io.mshrTask
  reqArb.io.refillBufRead_s2 <> refillBuf.io.r.req
  reqArb.io.releaseBufRead_s2 <> releaseBuf.io.r.req
  reqArb.io.putDataBufRead_s2 <> DontCare
  reqArb.io.fromMSHRCtl := mshrCtl.io.toReqArb
  reqArb.io.fromMainPipe := mainPipe.io.toReqArb
  reqArb.io.fromGrantBuffer := grantBuf.io.toReqArb
  reqArb.io.fromProbeHelper.blockSinkA := probeHelper.io.full
  reqArb.io.probeHelperTask <> probeHelper.io.task


  mshrCtl.io.fromReqArb.status_s1 := reqArb.io.status_s1
  mshrCtl.io.resps.sinkC := sinkC.io.resp
  mshrCtl.io.resps.sinkD := refillUnit.io.resp
  mshrCtl.io.resps.sinkE := grantBuf.io.e_resp
  mshrCtl.io.resps.sourceC := sourceC.io.resp
  mshrCtl.io.nestedwb := mainPipe.io.nestedwb
  mshrCtl.io.probeHelperWakeup := mainPipe.io.probeHelperWakeup


  directory.io.resp <> mainPipe.io.dirResp_s3
  directory.io.metaWReq <> mainPipe.io.metaWReq
  directory.io.tagWReq <> mainPipe.io.tagWReq

  dataStorage.io.req <> mainPipe.io.toDS.req_s3
  dataStorage.io.wdata := mainPipe.io.toDS.wdata_s3
  
  mainPipe.io.toMSHRCtl <> mshrCtl.io.fromMainPipe
  mainPipe.io.fromMSHRCtl <> mshrCtl.io.toMainPipe
  mainPipe.io.bufRead <> sinkC.io.bufRead
  mainPipe.io.bufResp <> sinkC.io.bufResp
  mainPipe.io.toDS.rdata_s5 := dataStorage.io.rdata
  mainPipe.io.toDS.error_s5 := dataStorage.io.error
  mainPipe.io.refillBufResp_s3.valid := RegNext(refillBuf.io.r.req.fire, false.B)
  mainPipe.io.refillBufResp_s3.bits := refillBuf.io.r.resp.bits
  mainPipe.io.releaseBufResp_s3.valid := RegNext(releaseBuf.io.r.req.fire, false.B)
  mainPipe.io.releaseBufResp_s3.bits := releaseBuf.io.r.resp.bits
  mainPipe.io.putDataBufResp_s3.valid := RegNext(putDataBuf.io.r.valid, false.B)
  mainPipe.io.putDataBufResp_s3.bits := putDataBuf.io.r.data
  mainPipe.io.fromReqArb.status_s1 := reqArb.io.status_s1
  mainPipe.io.putBufRead <> sinkA.io.pbRead
  mainPipe.io.putBufResp <> sinkA.io.pbResp
  mainPipe.io.clientDirConflict := probeHelper.io.dirConflict
  mainPipe.io.clientDirResp_s3 <> clientDirectory.io.resp
  mainPipe.io.fromReqBufSinkA.valid := a_reqBuf.io.out.valid
  mainPipe.io.fromReqBufSinkA.set := a_reqBuf.io.out.bits.set
  


  sinkA.io.fromMainPipe.putReqGood_s3 := mainPipe.io.toSinkA.putReqGood_s3
  sinkA.io.fromPutDataBuf.full := putDataBuf.io.full


  releaseBuf.io.w(0) <> sinkC.io.releaseBufWrite
  releaseBuf.io.w(0).bits.id := mshrCtl.io.releaseBufWriteId // id is given by MSHRCtl by comparing address to the MSHRs (only for ProbeAckData)
  releaseBuf.io.w(1) <> mainPipe.io.releaseBufWrite
  releaseBuf.io.w(2).valid := mshrCtl.io.nestedwbDataId.valid
  releaseBuf.io.w(2).bits.beat_sel := Fill(beatSize, 1.U(1.W))
  releaseBuf.io.w(2).bits.data := mainPipe.io.nestedwbData
  releaseBuf.io.w(2).bits.id := mshrCtl.io.nestedwbDataId.bits
  releaseBuf.io.w(2).bits.corrupt := false.B


  refillBuf.io.w(0) <> refillUnit.io.refillBufWrite
  refillBuf.io.w(1) <> mainPipe.io.refillBufWrite


  putDataBuf.io <> DontCare
//  putDataBuf.io.full // TODO: block sinkA
  putDataBuf.io.w <> mainPipe.io.putDataBufWrite
  putDataBuf.io.r.valid := reqArb.io.putDataBufRead_s2.valid
  putDataBuf.io.r.id := reqArb.io.putDataBufRead_s2.id

  sourceC.io.in <> mainPipe.io.toSourceC

  mshrCtl.io.grantStatus := grantBuf.io.grantStatus

  grantBuf.io.d_task <> mainPipe.io.toSourceD
  grantBuf.io.fromReqArb.status_s1 := reqArb.io.status_s1
  grantBuf.io.pipeStatusVec := reqArb.io.status_vec ++ mainPipe.io.status_vec
  mshrCtl.io.pipeStatusVec(0) := reqArb.io.status_vec(0) // s1 status
  mshrCtl.io.pipeStatusVec(1) := reqArb.io.status_vec(1) // s2 status
  mshrCtl.io.pipeStatusVec(2) := mainPipe.io.status_vec(0) // s3 status
  mshrCtl.io.fromReqArb.mshrTaskInfo <> reqArb.io.mshrTaskInfo

  sinkC.io.mshrStatus <> mshrCtl.io.toReqBuf
  sinkC.io.mshrFull := mshrCtl.io.toSinkC.mshrFull

  /* input & output signals */
  val inBuf = cacheParams.innerBuf
  val outBuf = cacheParams.outerBuf
  
  /* connect upward channels */
  sinkA.io.a <> inBuf.a(io.in.a)
  io.in.b <> inBuf.b(mshrCtl.io.sourceB)
  sinkC.io.c <> inBuf.c(io.in.c)
  io.in.d <> inBuf.d(grantBuf.io.d)
  grantBuf.io.e <> inBuf.e(io.in.e)

  /* connect downward channels */
  io.out.a <> outBuf.a(mshrCtl.io.sourceA)
  reqArb.io.sinkB <> outBuf.b(io.out.b)
  io.out.c <> outBuf.c(sourceC.io.out)
  refillUnit.io.sinkD <> outBuf.d(io.out.d)
  io.out.e <> outBuf.e(refillUnit.io.sourceE)

  dontTouch(io.in)
  dontTouch(io.out)

  topDownOpt.foreach (
    _ => {
      io.msStatus.get        := mshrCtl.io.msStatus.get
      io.dirResult.get.valid := RegNextN(directory.io.read.fire, 2, Some(false.B)) // manually generate dirResult.valid
      io.dirResult.get.bits  := directory.io.resp
    }
  )

  if (cacheParams.enablePerf) {
    val a_begin_times = RegInit(VecInit(Seq.fill(sourceIdAll)(0.U(64.W))))
    val timer = RegInit(0.U(64.W))
    timer := timer + 1.U
    a_begin_times.zipWithIndex.foreach {
      case (r, i) =>
        when (sinkA.io.a.fire() && sinkA.io.a.bits.source === i.U) {
          r := timer
        }
    }
    val d_source = grantBuf.io.d.bits.source
    val delay = timer - a_begin_times(d_source)
    val (first, _, _, _) = edgeIn.count(grantBuf.io.d)
    val delay_sample = grantBuf.io.d.fire && grantBuf.io.d.bits.opcode =/= ReleaseAck && first
    XSPerfHistogram(cacheParams, "a_to_d_delay", delay, delay_sample, 0, 20, 1, true, true)
    XSPerfHistogram(cacheParams, "a_to_d_delay", delay, delay_sample, 20, 300, 10, true, true)
    XSPerfHistogram(cacheParams, "a_to_d_delay", delay, delay_sample, 300, 500, 20, true, true)
    XSPerfHistogram(cacheParams, "a_to_d_delay", delay, delay_sample, 500, 1000, 100, true, false)
  }

  val dirRespBuffer = Module(new DirRespBuffer)
  val dirRespValid = RegNextN(reqArb.io.dirRead_s1.valid, 2, Some(false.B))
  dirRespBuffer.io.in.valid := dirRespValid
  dirRespBuffer.io.in.dirResp := directory.io.resp
  dirRespBuffer.io.in.clientDirResp := clientDirectory.io.resp
  dirRespBuffer.io.in.clientDirConflict := probeHelper.io.dirConflict
  dirRespBuffer.io.in.accept := mainPipe.io.acceptDirResp
  dontTouch(dirRespBuffer.io.out)

  mainPipe.io.dirResp_s3 := Mux(dirRespValid, directory.io.resp, dirRespBuffer.io.out.dirResp)
  mainPipe.io.clientDirResp_s3 := Mux(dirRespValid, clientDirectory.io.resp, dirRespBuffer.io.out.clientDirResp)
  mainPipe.io.clientDirConflict := Mux(dirRespValid, probeHelper.io.dirConflict, dirRespBuffer.io.out.clientDirConflict)

}
