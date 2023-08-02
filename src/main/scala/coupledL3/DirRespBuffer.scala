package coupledL3

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.BufferParams

class DirRespBuffer(implicit p: Parameters) extends L3Module {
    val io = IO(new Bundle{
        val in = Input(new Bundle{
            val dirResp = new DirResult
            val clientDirResp = new noninclusive.ClientDirResult
            val clientDirConflict = Bool()
            val valid = Bool()
            
            val accept = Bool()
        })

        val out = Output(new Bundle{
            val dirResp = new DirResult
            val clientDirResp = new noninclusive.ClientDirResult
            val clientDirConflict = Bool()
            val valid = Bool()
        })
    })
    
    val entries = 5 // TODO: Parameterize
    class DirInfo(implicit p: Parameters) extends L3Bundle {
        val dirResp = chiselTypeOf(io.in.dirResp)
        val clientDirResp = chiselTypeOf(io.in.clientDirResp)
        val clientDirConflict = chiselTypeOf(io.in.clientDirConflict)
    }
    val queue = Module(new Queue(new DirInfo, entries, pipe = false, flow = false))

    // queue.io.enq.ready
    queue.io.enq.valid := io.in.valid
    queue.io.enq.bits.dirResp := io.in.dirResp
    queue.io.enq.bits.clientDirResp := io.in.clientDirResp
    queue.io.enq.bits.clientDirConflict := io.in.clientDirConflict

    io.out.dirResp := queue.io.deq.bits.dirResp
    io.out.clientDirResp := queue.io.deq.bits.clientDirResp
    io.out.clientDirConflict := queue.io.deq.bits.clientDirConflict
    io.out.valid := queue.io.deq.valid

    queue.io.deq.ready := io.in.accept


    dontTouch(io)
}