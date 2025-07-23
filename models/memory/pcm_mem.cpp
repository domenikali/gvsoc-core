#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>
#include <stdio.h>

#include "pcm_mem.hpp"

Pcm::Pcm(vp::ComponentConf &config) : vp::Component(config) , event( this, Pcm::aimc_computation){
    
    traces.new_trace("trace", &trace, vp::DEBUG);

    // TODO: figure this out right, i don't think this can work at all but, if it does, it would be great
    // this->input_PCM.set_req_meth(&Pcm::req_PCM);
    // new_slave_port("input_pcm",&this->input_PCM);
    // this->input_AIMC.set_req_meth(&Pcm::req_AIMC);
    // new_slave_port("input_aimc",&this->input_AIMC);

    this->input_PCM_module.set_req_meth(&Pcm::req_PCM_module);
    new_slave_port("input",&this->input_PCM_module);

    this->power_ctrl_itf.set_sync_meth(&Pcm::power_ctrl_sync);
    new_slave_port("power_ctrl",&this->power_ctrl_itf);

    //initialize statistics parameters
    this->pcm_read_count=0;
    this->pcm_write_count.reset_count = 0;
    this->pcm_write_count.write_count = 0;
    this->aimc_compute_count=0;

    //initialize helper
    this->aimc_compute_helper = new aimc_compute_helper_t();

    //js config parameters
    js::Config *js_config = get_js_config()->get("power_trigger");
    this->powered_up = js_config != NULL && js_config->get_bool();

    this->Xi_size = this->get_js_config()->get("Xi_size")->get_int();
    this->cell_size=this->get_js_config()->get("cell_size")->get_int();
    this->mask= (1 << cell_size) - 1;
    this->output_size=this->get_js_config()->get("output_size")->get_int();
    this->cells_per_weight=this->get_js_config()->get("cells_per_weight")->get_int();
    
    this->tile_size=this->get_js_config()->get("tile_size")->get_int();
    this->array_size=this->get_js_config()->get("array_size")->get_int();
    this->n_sectors=this->get_js_config()->get("n_sectors")->get_int();
    this->matrix_length=array_size*tile_size;

    aimc_helper_inti(this->aimc_compute_helper, this->matrix_length, this->tile_size, this->array_size, this->n_sectors, this->cells_per_weight,this->cell_size,this->mask);

    this->pcm_width_log2=this->get_js_config()->get("pcm_width_log2")->get_int();
    this->input_width_log2=this->get_js_config()->get("input_width_log2")->get_int();
    this->output_width_log2=this->get_js_config()->get("output_width_log2")->get_uint();

    this->aimc_bus_freq = this->get_js_config()->get("aimc_bus_freq")->get_int();
    this->aimc_latency=this->get_js_config()->get("aimc_latency")->get_int();
    this->pcm_read_latency=this->get_js_config()->get("pcm_read_latency")->get_int();
    this->pcm_write_latency=this->get_js_config()->get("pcm_write_latency")->get_int();

    this->pcm_size = this->cells_per_weight*this->tile_size*this->array_size*this->n_sectors*this->matrix_length;
    this->mvm_full_result = new int64_t[this->matrix_length];

    this->pcm_output_size = (cell_size*this->cells_per_weight)%8>0?1:0;
    this->pcm_output_size += (cell_size*this->cells_per_weight-((cell_size*this->cells_per_weight)%8))/8;

    //number of bytes for each Yi value in the output vector (tells how many bytes are needed to store the output value using all it's aviable bits)
    this->response_byte_size = this->output_size%(sizeof(uint8_t))+1;
    this->response_byte_size+=((this->output_size - this->response_byte_size)/(sizeof(uint8_t)*8));

    //number of Xi adresses in bit is equal to the number of Xi values devided by the width of the input bus
    this->Xi_adresses = matrix_length/input_width_log2;
    //number of pcm cells adresses in bit is equal to the number of cells devided by the width of the pcm bus
    this->pcm_cells_adresses = (matrix_length*matrix_length)/pcm_width_log2;

    this->aimc_response = new uint8_t[this->matrix_length * this->response_byte_size];

    trace.msg("Builing PCM input vector (Size: 0x%x)\n",this->matrix_length);
    //input value array allocation
    this->input_vector = new input_size_t[this->matrix_length];
    if(this->input_vector==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: input vector allocation failed\n");
        throw std::bad_alloc();
    }
    memset(this->input_vector, 0,this->matrix_length*sizeof(input_size_t)); 

    trace.msg("Builing PCM Memory (Size: 0x%x, Cell Size in bytes: 0x%x)\n",this->pcm_size,sizeof(pcm_size_t));
    //matrix allocation
    this->pcm_cells = new pcm_size_t[this->pcm_size];
    if(this->pcm_cells==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: matrix allocation failed\n");
        throw std::bad_alloc();
    }
    //if memory is small, fill it with a special value to detect uninitialized variables
    if (this->pcm_size < (2<<24))
    {
        memset(this->pcm_cells, 0, this->pcm_size*sizeof(pcm_size_t));    
    }

    // sector matrix allocation
    this->sectors = new int8_t*[this->array_size];
    for(int i=0;i<this->array_size;i++){
        sectors[i] = new int8_t[this->n_sectors+1];
        for(int j=0;j<this->n_sectors+1;j++){
        sectors[i][j] = -1;
        }
    }

    //by default the AIMC is signed
    this->signed_computation = true;

    // Preload the Memory
    js::Config *stim_file_conf = this->get_js_config()->get("stim_file");
    if (stim_file_conf != NULL)
    {
        string path = stim_file_conf->get_str();
        if (path != "")
        {
            trace.msg("Preloading PCM with stimuli file (path: %s)\n", path.c_str());

            FILE *file = fopen(path.c_str(), "rb");
            if (file == NULL)
            {
                this->trace.fatal("Unable to open stim file: %s, %s\n", path.c_str(), strerror(errno));
                return;
            }
            if (fread(this->pcm_cells, 1, this->pcm_size, file) == 0)
            {
                this->trace.fatal("Failed to read stim file: %s, %s\n", path.c_str(), strerror(errno));
                return;
            }
        }
    }
}

vp::IoReqStatus Pcm::req_PCM_module(vp::Block *__this, vp::IoReq *req){
    //first address is for AIMC the remaining is for PCM

    
    Pcm *_this = (Pcm *)__this;
    _this->trace.msg("req size: 0x%x\n",req->get_size());
    if(req->get_addr() < (_this->matrix_length +sizeof(uint32_t))){//32_t size of cmd reg
        return Pcm::req_AIMC(__this, req);
    }
    else{
        req->set_addr(req->get_addr() - (_this->matrix_length + sizeof(uint32_t)));
        return Pcm::req_PCM(__this, req);
    }
    return vp::IO_REQ_INVALID;
}

vp::IoReqStatus Pcm::req_PCM(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;
    _this->trace.msg(vp::Trace::LEVEL_INFO,"PCM: Request received, addr: 0x%x, size: %d, is_write: %d\n",req->get_addr(),req->get_size(),req->get_is_write());

    if(req->get_is_write()){
        req->inc_latency(_this->pcm_write_latency);
        return _this->handle_PCM_write(req->get_addr(), req->get_size(),req->get_data());
    }
    else{
        req->inc_latency(_this->pcm_read_latency);
        _this->pcm_read_count++;
        return _this->handle_PCM_read(req->get_addr(), req->get_size(),req->get_data());
    }

}

vp::IoReqStatus Pcm::handle_PCM_read(uint64_t addr, uint64_t size, uint8_t *data){
    if(addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: read out of bounds\n");
        return vp::IO_REQ_INVALID;
    }

    size_t output_buffer_size = 4;
    //uint8_t * pcm_data = new uint8_t[output_buffer_size];

    // for(int i=0;i<output_buffer_size;i++){
    //     this->pcm_load(addr+i,this->pcm_cells,&pcm_data[i*this->pcm_output_size]);
    // }

    //read the PCM value from the memory
    //uint64_t pcm_value = 1000;
    memcpy((void *)data, (void*)(this->pcm_cells+addr), output_buffer_size);
    //delete[] pcm_data;
    return vp::IO_REQ_OK;
}

void Pcm::pcm_load(uint64_t index,pcm_size_t * matrix,uint8_t * byteStream){
    uint64_t value = 0;

    for(int i=0;i<this->cells_per_weight;i++){
        //std::cout<<std::bitset<cell_size>(matrix[index*nCells+i])<<std::endl;
        value |= (matrix[index*this->cells_per_weight+i]&this->mask) << ((this->cells_per_weight-i-1)*cell_size);
    }
    //std::cout<<std::bitset<64>(value)<<std::endl;

    for(int i=0;i<this->pcm_output_size;i++){
        byteStream[this->pcm_output_size-i-1] = value >> (i*8) & 0xFF;
    }

}

//FIXEME: this is valid only for 1<=cell_size<=8
vp::IoReqStatus Pcm::handle_PCM_write(uint64_t addr, uint64_t size, uint8_t *data){
    if(addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: write out of bounds\n");
        return vp::IO_REQ_INVALID;
    }
    size_t buff_size;
    if(this->cell_size<=8){
        buff_size = size;
    }
    else{
        buff_size = (size+this->cell_size-1)/this->cell_size;
    }
    pcm_size_t *buff= (pcm_size_t *)malloc(buff_size*sizeof(pcm_size_t));
    if(buff==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: write allocation failed\n");
        return vp::IO_REQ_INVALID;
    }

    for(size_t i=0;i<buff_size;i++){
        //this->trace.msg(vp::Trace::LEVEL_DEBUG,"PCM: writing cell %d, value: 0x%x\n",i,data[i]);
        buff[i]=data[i];
    }
    memcpy((void *)(this->pcm_cells+addr), (void *)buff, buff_size*sizeof(pcm_size_t));
    free(buff);
    return vp::IO_REQ_OK;

    
}

vp::IoReqStatus Pcm::req_AIMC(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;
    _this->trace.msg(vp::Trace::LEVEL_INFO,"AIMC: Request received, addr: 0x%x, size: %d, is_write: %d\n",req->get_addr(),req->get_size(),req->get_is_write());
    
    //if the request is a write it handles it
    if(req->get_is_write()&&req->get_addr()!=0){
        return _this->handle_Xi_write(req);
    }

    if(!req->get_is_write()&&req->get_addr()!=0){
        return _this->handle_Yi_read(req);
    }
    
    //if the request is not a write handles different commands
    uint32_t cmd;
    for(int i=3;i>=0;i--){
        cmd = (cmd<<8)+req->get_data()[i];
        //_this->trace.msg("AIMC: Command byte %d: 0x%x\n", i, req->get_data()[i]);
    }
    
    
    _this->trace.msg("AIMC: Command received: 0x%lx\n", cmd);
    switch(cmd&(uint32_t)CMD::CMD_MASK){
        case (uint32_t)CMD::CMD_COMPUTE:
            _this->trace.msg("AIMC: Compute command\n");
            return _this->handle_AIMC_compute(req);

        case (uint32_t)CMD::CMD_PARAM:
            _this->trace.msg("AIMC: Change configuration settings\n");
            _this->aimc_settings(cmd);
            return vp::IoReqStatus::IO_REQ_OK;

        case (uint32_t)CMD::CMD_ABORT:
            _this->trace.msg("AIMC: Computation aborted\n");
            _this->aimc_abort();
            return vp::IoReqStatus::IO_REQ_OK;

        default:
            _this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: Unknown command settings 0x%x\n",cmd);
    }
    //if not recognized
    return vp::IoReqStatus::IO_REQ_INVALID;
}

vp::IoReqStatus Pcm::handle_Xi_write(vp::IoReq *req){
    uint64_t addr = req->get_addr()-0x4;//req cmd offset (32 bits or sizeof(uint32_t))
    uint64_t size = req->get_size();//assuming size in bytes
    uint8_t *data = req->get_data();

    if(size>this->input_width_log2){
        size = this->input_width_log2;
        this->trace.msg(vp::Trace::LEVEL_WARNING,"AIMC: Xi write size too big, max size is %d bytes, data croped\n",this->input_width_log2);
    }
    if(addr>=this->matrix_length||addr+size>this->matrix_length){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: Xi write out of bounds, addr: 0x%x, size: %d\n",addr,size);
        return vp::IO_REQ_INVALID;
    }

    this->trace.msg("AIMC: Writing Xi values at address 0x%x, size %d\n", addr, size);
    memcpy(&this->input_vector[addr], data, size);

    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_Yi_read(vp::IoReq *req){

    uint64_t addr = req->get_addr()-0x4; //req cmd offset (32 bits or sizeof(uint32_t))
    uint64_t size = req->get_size();//assuming size in bytes
    uint8_t *data = req->get_data();

    if(size>this->output_width_log2){
        size = this->output_width_log2;
        this->trace.msg(vp::Trace::LEVEL_WARNING,"AIMC: Xi write size too big, max size is %d bytes, data croped\n",this->input_width_log2);
    }

    this->trace.msg("AIMC: Reading Yi values at address 0x%x, size %d\n", addr, size);

    memcpy(&this->aimc_response[addr],data,size);

    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::aimc_settings(uint32_t cmd){

    switch (cmd&(uint32_t)CMD_SETTINGS::CMD_SETTINGS_MASK)
    {
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_SECTORS:
        this->enabled_sectors(cmd);
        return vp::IO_REQ_OK;
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_U:
        this->signed_computation=false;
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_U_DOUBLE_WEIGHT:
        this->signed_computation=false;
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_T_STEP_S:
        this->signed_computation=true;
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_S_DOUBLE_WEIGHT:
        this->signed_computation=true;
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_SINGLE_STEP:
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_FAST_SINGLE_STEP:
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_INPUT_PRECISION:
        /* code */
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_BL:
        /* code */
        break;

    }
    //if not recognized (to avoid warnings)    
    this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: Unknown command settings 0x%x\n",cmd);
    return vp::IO_REQ_INVALID;
}

vp::IoReqStatus Pcm::handle_AIMC_compute(vp::IoReq *req){
    
    if(!this->event.is_enqueued()){
        this->trace.msg("AIMC: computation enqued\n");
        this->pending_req = req;
        
        this->mvm_multithreaded(this->pcm_cells, this->input_vector, this->sectors, this->mvm_full_result);
        this->convert_to_adc(this->mvm_full_result, this->aimc_response);
        this->event.enqueue(this->aimc_latency);

        return vp::IO_REQ_PENDING;

    }
    return vp::IO_REQ_INVALID;
}

void Pcm::aimc_computation(vp::Block* __this, vp::ClockEvent *event){
    Pcm *_this = (Pcm *)__this;
    _this->trace.msg("AIMC: computation result returned\n");
    
    _this->pending_req->data = _this->aimc_response;
    _this->pending_req->get_resp_port()->resp(_this->pending_req);
}

//TODO: kill threads??
void Pcm::aimc_abort(){


}

inline uint64_t Pcm::index(aimc_compute_helper_t * helper,int sect,int tile, int row, int column, int cell){
    return ((((sect*helper->array_size)+tile)*helper->tile_size+row)*helper->matrix_length+column)*helper->cells_per_weight+cell;
}

void Pcm::compute_flat_mvm_tile(aimc_compute_helper_t * helper,pcm_size_t *matrix, input_size_t * vector, int64_t * result, int8_t *sector, int tile, int i,int inc){
    for(int j=i;j<i+inc;j++){
        for(int k=0;k<helper->matrix_length;k++){
            //create an all-zero int64_t
            int64_t weight=static_cast<int64_t>(0);
            uint8_t y=0;
            //create an indexing for the bit shift
            int bit_index=helper->max_shift;
            
            while(sector[y]>=0){
                for(int x=0;x<helper->cells_per_weight;x++){
                    //generate the index of the flat 5D array
                    uint64_t idx = index(helper,sector[y],i,j,k,x);
                    //the mask is used to select the bits of the cell, then it's shifted to the right position
                    weight |= ((matrix[idx]&helper->mask)<<bit_index);
                    //the bit index is decremented by the size of the cell
                    bit_index-=helper->cell_size;   
                }
                y++;
            }
            //check if the weight is negative
            if((weight&helper->negative)!=0){
                //if the weight is negative make the 2's complement of it
                weight = static_cast<int64_t>(static_cast<uint64_t>(weight) | helper->negative_mask);
            }
            //write the result into the result vector
            result[tile*helper->tile_size+j] += weight * vector[k];
        }
    }
}

  
void Pcm::mvm_multithreaded(pcm_size_t* matrix, input_size_t * vector, int8_t **sector,int64_t * result){
    //result vector initialization
    memset(result,0,sizeof(int64_t)*this->matrix_length);
    //threads vector
    std::vector<std::thread> threads;
    //loops, each thread will compute a part of the matrix tile 
    for(int i=0;i<this->array_size;i++){
        for(int j=0;j<2;j++){
            std::thread t(compute_flat_mvm_tile,this->aimc_compute_helper, matrix, vector, result, sector[i], i,j*64, 64);
            threads.push_back(move(t));
        }
    }
    //thread join, not necesary but highly raccomended for extensive mvm use (E.G: it's fine without it as long as the ammount of request is low)
    for(int i=0;i<threads.size();i++){
        threads[i].join();
    }
}

void Pcm::convert_to_adc(int64_t *input, uint8_t *output){
    //convert the input vector to the output vector
    for(int i=0;i<this->matrix_length;i++){
        this->adc(input[i],&output[i*this->response_byte_size]);
    }
}

void Pcm::adc(int64_t input,uint8_t * output) {

    uint64_t max_mask = (~0ULL << this->output_size+1)>>1;
    uint64_t min_mask = (~0ULL << this->output_size);

    //clipping
    if(input & max_mask !=0) {
        input = ~(~1ULL << this->output_size+1);
    }else if(input & min_mask !=0) {
        input = ~(~1ULL << this->output_size);
    }
    
    //output buffer initialization, loads the less significant byte first at the end of the array
    for (int i = 0; i < this->response_byte_size; ++i) {
        output[this->response_byte_size-i-1] = input >> (i * 8) & 0xFF;
    }  
    //most important byte masking to match the output size
    uint8_t mask=0xFF;
    int u_conversion = this->signed_computation ? 0 : 1; 
    mask>>=(8-(this->output_size)+u_conversion)%8;
    //masking most significant byte
    output[0] &= mask;
}

void Pcm::enabled_sectors(uint32_t configuration){
    
    uint32_t config_mask = 0x00800000;
    this->used_sectors = 0;

    this->trace.msg("AICM: sector enable: \n");

    for(int i=0;i<this->array_size;i++){
        int k=0;
        for(int j=0;j<n_sectors;j++){
            if((configuration&config_mask) != 0){
                this->sectors[i][k++] = j;
                this->trace.msg("array:[%d], sector[%d]\n",i,j);
            }
        
            config_mask = config_mask >> 1;
        }


        if(k>used_sectors){
            used_sectors = k;
        }
        sectors[i][k] = -1; // End of sector marker
    }
    
    this->negative_mask =(~0ULL << (this->cell_size*used_sectors*this->cells_per_weight));
    this->negative = 1 << ((this->cell_size*used_sectors*this->cells_per_weight)-1); 
    this->max_shift=(used_sectors*this->cells_per_weight*cell_size )-cell_size;
    aimc_helper_param(this->aimc_compute_helper, this->negative_mask, this->negative, this->max_shift);
  
}

void Pcm::free_component(){
    delete[] this->input_vector;
    delete[] this->pcm_cells;
    delete[] this->mvm_full_result;
    delete[] this->aimc_response;
    for(int i=0;i<this->array_size;i++){
        delete[] this->sectors[i];
    }
    delete[] this->sectors;
    delete this->aimc_compute_helper;
}

void Pcm::power_ctrl_sync(vp::Block *__this, bool value)
{
    Pcm *_this = (Pcm *)__this;
    _this->powered_up = value;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Pcm(config);
}