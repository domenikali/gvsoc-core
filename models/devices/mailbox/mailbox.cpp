#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>
#include <stdio.h>
#include <string.h>

class Mailbox : public vp::Component
{

    public:
    Mailbox(vp::ComponentConf &config);


    static vp::IoReqStatus req(vp::Block *__this, vp::IoReq *req);  

    private:
    vp::Trace trace;
    vp::IoSlave in;

};

Mailbox::Mailbox(vp::ComponentConf &config)
    : vp::Component(config)
{
    this->traces.new_trace("trace", &trace, vp::DEBUG);
    in.set_req_meth(&Mailbox::req);
    new_slave_port("input", &in);

    this->new_slave_port("input", &this->in);
    this->in.set_req_meth(&Mailbox::req);
}

vp::IoReqStatus Mailbox::req(vp::Block *__this, vp::IoReq *req){
    Mailbox *_this = (Mailbox *)__this;
    _this->trace.msg("Received request: addr=0x%lx size=%lu is_write=%d\n", req->get_addr(), req->get_size(), req->get_is_write());
    return vp::IoReqStatus::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Mailbox(config);
}