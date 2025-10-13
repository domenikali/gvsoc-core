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
    this->n_layers=this->get_js_config()->get("n_layers")->get_int();
    this->n_sectors=this->get_js_config()->get("n_sectors")->get_int();
    this->matrix_length=array_size*tile_size;


    this->pcm_width_log2=this->get_js_config()->get("pcm_width_log2")->get_int();
    this->input_width_log2=this->get_js_config()->get("input_width_log2")->get_int();
    this->output_width_log2=this->get_js_config()->get("output_width_log2")->get_uint();

    this->aimc_bus_freq = this->get_js_config()->get("aimc_bus_freq")->get_int();
    this->aimc_latency=this->get_js_config()->get("aimc_latency")->get_int();
    this->pcm_read_latency=this->get_js_config()->get("pcm_read_latency")->get_int();
    this->pcm_write_latency=this->get_js_config()->get("pcm_write_latency")->get_int();

    this->pcm_size = this->matrix_length * this->matrix_length * this->n_layers;
    this->mvm_full_result = new int64_t[this->matrix_length];
    if(this->mvm_full_result==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: MVM result allocation failed\n");
        throw std::bad_alloc();
    }

    this->pcm_output_size = (cell_size*this->cells_per_weight)%8>0?1:0;
    this->pcm_output_size += (cell_size*this->cells_per_weight-((cell_size*this->cells_per_weight)%8))/8;

    //number of bytes for each Yi value in the output vector (tells how many bytes are needed to store the output value using all it's aviable bits)
    this->response_byte_size = this->output_size%(sizeof(uint8_t))+1;
    this->response_byte_size+=((this->output_size - this->response_byte_size)/(sizeof(uint8_t)*8));
    this->trace.msg("PCM: response byte size: %d\n",this->response_byte_size);

    //number of Xi adresses in bit is equal to the number of Xi values devided by the width of the input bus
    this->Xi_adresses = matrix_length/input_width_log2;
    //number of pcm cells adresses in bit is equal to the number of cells devided by the width of the pcm bus
    this->pcm_cells_adresses = (matrix_length*matrix_length)/pcm_width_log2;

    this->aimc_response = new uint8_t[this->matrix_length * this->response_byte_size];
    if(aimc_response==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: AIMC response allocation failed\n");
        throw std::bad_alloc();
    }

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
    if (true||this->pcm_size < (2<<24))
    {
        memset(this->pcm_cells, 0, this->pcm_size*sizeof(pcm_size_t));    
    }

    // sector matrix allocation
    this->sectors = new int8_t[this->n_sectors+1];
    
    if(this->sectors==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: sectors allocation failed\n");
        throw std::bad_alloc();
    }
    this->layers=new int8_t*[this->n_sectors+1];
    if(this->layers==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: layers allocation failed\n");
        throw std::bad_alloc();
    }
    for(int i=0;i<this->n_sectors+1;i++){
        this->layers[i]=new int8_t[this->n_layers+1];
        this->layers[i][0]=0;
        this->layers[i][1]=-1;
    }

    //by default the AIMC is signed
    this->trace.msg("Default AIMC operation: signed computation\n");
    this->signed_computation=true;
    this->precision_mask = (1<<(sizeof(input_size_t)*8))-1;
    this->trace.msg("AIMC: Input default precision set to %s bits\n",std::bitset<8>(precision_mask).to_string().c_str());

    this->mvm_threads=this->get_js_config()->get("mvm_threads")->get_int();

    if(this->mvm_threads<1||(mvm_threads & (mvm_threads - 1))){
        this->mvm_threads=1;
        this->trace.msg(vp::Trace::LEVEL_WARNING,"PCM: mvm_threads must be a power of 2, setting to 1\n");
    }
    this->trace.msg("PCM: Using %d threads for each sector/layer for MVM operations\n",this->mvm_threads);

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
    Pcm *_this = (Pcm *)__this;
    //_this->trace.msg("req size: 0x%x\t req addr: 0x%x\n",req->get_size(),req->get_addr());
    
    if(req->get_addr() < (_this->matrix_length +sizeof(uint32_t))){//32_t size of cmd reg + Xi vector
        return Pcm::req_AIMC(__this, req);
    }
    else{//not required PCM direct access
        req->set_addr(req->get_addr() - (_this->matrix_length + sizeof(uint32_t)));
        return Pcm::req_PCM(__this, req);
    }
    return vp::IO_REQ_INVALID;
}

vp::IoReqStatus Pcm::req_PCM(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;
    //_this->trace.msg(vp::Trace::LEVEL_INFO,"PCM: Request received, addr: 0x%x, size: %d, is_write: %d\n",req->get_addr(),req->get_size(),req->get_is_write());

    if(req->get_is_write()){
        req->inc_latency(_this->pcm_write_latency);
        return _this->handle_PCM_write(req);
    }
    else{
        req->inc_latency(_this->pcm_read_latency);
        _this->pcm_read_count++;//increment pcm read count for performance statistics
        return _this->handle_PCM_read(req);
    }

}

vp::IoReqStatus Pcm::handle_PCM_read(vp::IoReq *req){
    if(req->addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: read out of bounds\n");
        return vp::IO_REQ_INVALID;
    }
    
    memcpy(req->data, (void*)(this->pcm_cells+req->addr), req->get_size());
    //this->trace.msg(vp::Trace::LEVEL_DEBUG,"PCM: read value: 0x%x\n",*(uint32_t *)data);
    return vp::IO_REQ_OK;
}

//FIXEME: this is valid only for 1<=cell_size<=8
vp::IoReqStatus Pcm::handle_PCM_write(vp::IoReq *req){
    uint64_t addr = req->get_addr();
    size_t size = req->get_size();
    if(addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: write out of bounds\n");
        return vp::IO_REQ_INVALID;
    }
    
    //this->trace.msg(vp::Trace::LEVEL_DEBUG,"PCM: write values 0x%x \n",*(uint32_t*)buff);
    memcpy((void *)(this->pcm_cells+addr), req->data, size);
    
    // for(int i=0;i<4;i++){
    //     this->trace.msg(vp::Trace::LEVEL_DEBUG,"PCM: Writing PCM values at address 0x%x, 0x%x\n", addr+i, this->pcm_cells[addr+i]);
    // }
    
    return vp::IO_REQ_OK;

    
}

vp::IoReqStatus Pcm::req_AIMC(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;
    //_this->trace.msg(vp::Trace::LEVEL_INFO,"AIMC: Request received, addr: 0x%x, size: %d, is_write: %d\n",req->get_addr(),req->get_size(),req->get_is_write());
    
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
    return vp::IoReqStatus::IO_REQ_INVALID;
}

vp::IoReqStatus Pcm::handle_Xi_write(vp::IoReq *req){
    uint64_t addr = req->get_addr()-0x4;//req cmd offset (32 bits or sizeof(uint32_t))
    uint64_t size = req->get_size();//assuming size in bytes

    if(size>this->input_width_log2){
        size = this->input_width_log2;
        this->trace.msg(vp::Trace::LEVEL_WARNING,"AIMC: Xi write size too big, max size is %d bytes, data croped\n",this->input_width_log2);
    }
    if(addr>=this->matrix_length||addr+size>this->matrix_length){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: Xi write out of bounds, addr: 0x%x, size: %d\n",addr,size);
        return vp::IO_REQ_INVALID;
    }

    //this->trace.msg("AIMC: Writing Xi values at address 0x%x, size %d, value 0x%x\n", addr, size, *(uint32_t *)req->data);
    //this->trace.msg("AIMC: Writing Xi values: %x\n", *(uint32_t *)data);
    memcpy(&this->input_vector[addr], req->data, size);
    

    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_Yi_read(vp::IoReq *req){

    uint64_t addr = req->get_addr()-0x4; //req cmd offset (32 bits or sizeof(uint32_t))
    uint64_t size = req->get_size();//assuming size in bytes

    if(size>this->output_width_log2){
        size = this->output_width_log2;
        this->trace.msg(vp::Trace::LEVEL_WARNING,"AIMC: Xi write size too big, max size is %d bytes, data croped\n",this->input_width_log2);
    }

    memcpy(req->data,&this->aimc_response[addr],size);
    this->trace.msg("AIMC: Reading Yi values at address 0x%x, size %d, value 0x%x\n", addr, size, *(uint32_t *)&this->aimc_response[addr]);


    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::aimc_settings(uint32_t cmd){

    switch (cmd&(uint32_t)CMD_SETTINGS::CMD_SETTINGS_MASK)
    {
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_SECTORS:
        this->configuration_registers=cmd;
        this->enabled_sectors(cmd);
        if(!this->enabled_layers(cmd,false)) return vp::IO_REQ_INVALID;
        
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_U:
        this->signed_computation=false;
        if(!this->enabled_layers(configuration_registers,false)) return vp::IO_REQ_INVALID;
        this->trace.msg("AIMC: Unsigned two step computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_U_DOUBLE_WEIGHT:
        this->signed_computation=false;
        if(!this->enabled_layers(cmd,true)) return vp::IO_REQ_INVALID;
        this->trace.msg("AIMC: Two step unsigned double weight computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_S:
        this->signed_computation=true;
        if(!this->enabled_layers(configuration_registers,false)) return vp::IO_REQ_INVALID;
        this->trace.msg("AIMC: Two step signed computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_TWO_STEP_S_DOUBLE_WEIGHT:
        this->signed_computation=true;
        if(!this->enabled_layers(cmd,true)) return vp::IO_REQ_INVALID;
        this->trace.msg("AIMC: Two step signed double weight computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_SINGLE_STEP:
        this->signed_computation=true;
        if(!this->enabled_layers(configuration_registers,false)) return vp::IO_REQ_INVALID;
        this->trace.msg("AIMC: Signed single step computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_FAST_SINGLE_STEP:
        this->signed_computation=true;
        this->trace.msg("AIMC: Signed fast single step computation\n");
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_INPUT_PRECISION:
        this->set_precision(cmd);
        break;
    case (uint32_t)CMD_SETTINGS::CMD_SETTINGS_BL:
        break;

    default:
        this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: Unknown command settings 0x%x\n",cmd);
        return vp::IO_REQ_INVALID;
    }
    return vp::IO_REQ_OK;
    
}

void Pcm::set_precision(uint32_t cmd){
    this->precision_mask = (1<<((cmd&0x00000007)+(this->signed_computation?1:0)))-1;
    this->trace.msg("AIMC: Input precision set to %s bits\n",std::bitset<8>(this->precision_mask).to_string().c_str());
}

vp::IoReqStatus Pcm::handle_AIMC_compute(vp::IoReq *req){
    
    if(!this->event.is_enqueued()){
        
        this->trace.msg("AIMC: computation enqued\n");
        this->pending_req = req;
        this->mvm(this->pcm_cells, this->input_vector, this->layers,this->sectors, this->mvm_full_result);
        this->aimc_compute_count++;//increment aimc compute count for performance statistics
        this->trace.msg("AIMC: MVM computation done\n");
        this->adc(this->mvm_full_result, this->aimc_response);
        
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


void Pcm::optimised_singlethreaded(pcm_size_t* matrix, input_size_t* vector,  int8_t**  layers, int8_t* sectors, int64_t* result) {
    memset(result, 0, 512 * sizeof(int64_t)); 

    int s_idx = 0;
    while (sectors[s_idx] != -1) {
        int s = sectors[s_idx];

        int l_idx = 0;
        int8_t* curr_layers = layers[s];
        while (curr_layers[l_idx] != -1) {
            int l = curr_layers[l_idx];
            pcm_size_t* matrix_base = &matrix[inedx(s, l, 0, 0)];

            for (int y = 0; y < this->tile_size; ++y) {
                int64_t row_sum = 0;

                for (int x = 0; x < this->matrix_length; ++x) {
                    row_sum += *matrix_base * vector[x];
                    matrix_base++; 
                }
                result[s * this->tile_size + y] += row_sum;
            }
            l_idx++;
        }
        s_idx++;
    }
}

void Pcm::optimised_multithreaded(pcm_size_t* matrix, input_size_t* vector,  int8_t** layers, int8_t* sectors, int64_t* result) {
    memset(result, 0, 512 * sizeof(int64_t)); 
    std::vector<std::thread> threads;
    int thread_count = 1;
    std::atomic<int64_t> temp_result[512];
    for(int i=0;i<512;++i)temp_result[i]=0;
    
    int s_idx = 0;
    while (sectors[s_idx] != -1) {
        int s=sectors[s_idx];
        int l_idx = 0;
        int8_t* curr_layers = layers[s];
        while (curr_layers[l_idx] != -1) {
            int l = curr_layers[l_idx];

            for(int i=0;i<thread_count;++i){
                int y_start = i * (tile_size / thread_count);
                int end = (i + 1) * (tile_size / thread_count); 
                
                std::thread t(multithreaded_worker_thread, &matrix[inedx(s, l, 0, 0)+y_start*this->matrix_length],s, vector, temp_result,y_start,end,this->matrix_length);
                threads.push_back(move(t));
            }
            l_idx++;
        }
        s_idx++;
    }
    for(int i=0;i<threads.size();i++){
        threads[i].join();
    }
    for(int i=0;i<512;++i){
        result[i]=temp_result[i].load(std::memory_order_relaxed);
    }
}


void Pcm::multithreaded_worker_thread( pcm_size_t* matrix, int8_t s,input_size_t* vector, std::atomic<int64_t>* result,int y_start,int end,int matrix_length) {
    for (int y = y_start; y < end; ++y) {
        int64_t row_sum = 0;

        for (int x = 0; x < matrix_length; ++x) {
            
            row_sum += *matrix * vector[x];       
            matrix++; 
        }
        result[s * 128 + y].fetch_add(row_sum, std::memory_order_relaxed);
    }
}


void Pcm::mvm(pcm_size_t*matrix,input_size_t*vector, int8_t** layers, int8_t*sectors,int64_t*result ){
    #ifdef REQUIRE_THREADS
        this->trace.msg("AIMC: Threads required, using multi-threaded MVM\n");
        optimised_multithreaded(matrix, vector, layers, sectors, result);
    #else    
        #ifdef __OPTIMIZE__
            this->trace.msg("AIMC: Compiler Optimisation enabled, using single-threaded MVM\n");
            optimised_singlethreaded(matrix, vector, layers, sectors, result);  
        #else
            
            if (used_sectors <= 1) {
                this->trace.msg("AIMC: Dispatch light computation, using single-threaded MVM\n");
                optimised_singlethreaded(matrix, vector, layers, sectors, result);
                return;
            } 
            this->trace.msg("AIMC: Dispatch heavy computation, using multi-threaded MVM\n");
            optimised_multithreaded(matrix, vector, layers, sectors, result);
            
        #endif
    #endif
}

// void Pcm::compute_flat_mvm_tile(aimc_compute_helper_t * helper,pcm_size_t *matrix, input_size_t * vector, int64_t * result, int8_t *sector, int tile, int i,int inc){
//     for(int j=i;j<i+inc;j++){
//         for(int k=0;k<helper->matrix_length;k++){
//             //create an all-zero int64_t
//             int64_t weight=static_cast<int64_t>(0);
//             uint8_t y=0;
//             //create an indexing for the bit shift
//             int bit_index=helper->max_shift;
            
//             while(sector[y]>=0){
//                 for(int x=0;x<helper->cells_per_weight;x++){
//                     //generate the index of the flat 5D array
//                     uint64_t idx = index(helper,sector[y],tile,j,k,x);
//                     //the mask is used to select the bits of the cell, then it's shifted to the right position
//                     weight |= ((matrix[idx]&helper->mask)<<bit_index);
//                     //the bit index is decremented by the size of the cell
//                     bit_index-=helper->cell_size;   
//                 }
//                 y++;
//             }
//             //check if the weight is negative
//             if((weight&helper->negative)!=0){
//                 //if the weight is negative make the 2's complement of it
//                 weight = static_cast<int64_t>(static_cast<uint64_t>(weight) | helper->negative_mask);
//             }
//             //write the result into the result vector
//             result[tile*helper->tile_size+j] += weight * (vector[k]&helper->precision_mask);
//         }
//     }
// }

// void Pcm::compute_flat_differential_tile(aimc_compute_helper_t * helper,pcm_size_t *matrix, input_size_t * vector, int64_t * result, int8_t *sector, int tile, int i,int inc){
//     for(int j=i;j<i+inc;++j){
//         for(int k=0;k<helper->matrix_length;++k){
//             //create an all-zero int64_t
//             int64_t weight=static_cast<int64_t>(0);
//             uint8_t y=0;
//             //create an indexing for the bit shift
            
//             while(sector[y]>=0){

//                 for(int x=1;x<helper->cells_per_weight;++x){
//                     //generate the index of the flat 5D array
//                     uint64_t idx = index(helper,sector[y],tile,j,k,x);
//                     //the mask is used to select the bits of the cell
//                     weight |= ((matrix[idx]&helper->mask));
                    
//                 }
//                 weight = matrix[index(helper,sector[y],tile,j,k,0)] & ~weight;
//                 y++;
//             }
//             //check if the weight is negative
//             if((weight&helper->negative)!=0){
//                 //if the weight is negative make the 1's complement of it
//                 weight = ~weight;
//             }
//             //write the result into the result vector
//             result[tile*helper->tile_size+j] += weight * vector[k];
//         }
//     }
// }

  
// void Pcm::mvm_multithreaded(pcm_size_t* matrix, input_size_t * vector, int8_t **sector,int64_t * result){
//     //result vector initialization
//     memset(result,0,sizeof(int64_t)*this->matrix_length);
//     //threads vector
//     std::vector<std::thread> threads;
//     //loops, each thread will compute a part of the matrix tile 
//     uint32_t threads_per_tile =4; //E.G. 16 thread total 
//     for(int i=0;i<this->array_size;i++){
//         for(int j=0;j<threads_per_tile;j++){
//             std::thread t(mvm_worker,this->aimc_compute_helper, matrix, vector, result, sector[i], i,j*(tile_size/threads_per_tile), tile_size/threads_per_tile);
//             threads.push_back(move(t));
//         }
//     }
//     //thread join, not necesary but highly raccomended for extensive mvm use (E.G: it's fine without it as long as the ammount of request is low)
//     for(int i=0;i<threads.size();i++){
//         threads[i].join();
//     }
//     this->trace.msg("AIMC: MVM computation completed\n");
    
// }

//FIXME: It works only for output size <=8 bit
void Pcm::adc(int64_t *input, uint8_t *output){
    
    this->trace.msg("AIMC: converting input vector to output vector\n");
    uint64_t positive_mask = (~0ULL << this->output_size);
    positive_mask>>=1;
    uint64_t negative_mask = (~0ULL << this->output_size-(signed_computation?0:1));
    uint64_t m = 1ULL<<63;    
    //most important byte masking to match the output size
    uint8_t mask=0xFF;
    if(!signed_computation){
        mask>>=1;
    }
    
    for(int i=0;i<this->matrix_length;i++){
        if(i==0){
            this->trace.msg("AIMC: input[%d]: 0b%s\n",i,std::bitset<64>(input[i]).to_string().c_str());
        }
        int64_t val = input[i];
        //clipping
        if(!signed_computation){
            if((val&m)!=m){
                if((val&negative_mask)!=0){
                    val=~(~1ULL <<output_size-2);
                }    
            }
            else{
                val=~val+1;
                if(input[i]!=0&&(val&0xFF)==0){
                    val=~(~1ULL <<output_size-2);
                }
            }
            
        }
        else if((val&m)!=0){
            if((val & negative_mask)!=negative_mask){
                val =0;
                val=1<<output_size-1;
            }
        }
        else if((val & positive_mask )!=0) {
            val = ~(~1ULL << this->output_size-2);
        }
        
        output[i]=val;
        
        //masking most significant byte
        output[i] &= mask;
    }
}


bool Pcm::enabled_layers(uint32_t configuration,bool two_setp){
    if(!two_setp){
        //find the first layer enabled and set it for all the sectors
        uint32_t layer_mask=0x00000001;
        uint8_t i=0;
        while((layer_mask&configuration) == 0){
            layer_mask = layer_mask << 1;
            i++;
        }
        if(i>=8){
            this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: single step computation selected but no layer enabled\n");
            return false;
        }
        this->trace.msg("AIMC: single step computation selected, using layer %d\n",i);
        for(int j=0;j<n_sectors&&sectors[j]!=-1;j++){
            
            this->layers[j][0]=i;
            this->layers[j][1]=-1;
        }
        return true;
    }
    //find the first (i) and the second (k) layer enabled and set it for all the sectors in the same order bottom up
    int i=0;
    uint32_t layer_mask=0x00000002;
    while((layer_mask&configuration) == 0){
        layer_mask = layer_mask << 1;
        i++;
    }
    if(i>=8){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: two step double weight computation selected but no layer enabled\n");
        return false;
    }
    int k=0;
    layer_mask=0x00000100;
    while((layer_mask&configuration) == 0){
        layer_mask = layer_mask << 1;
        k++;
    }
    if(k>=8){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"AIMC: two step double weight computation selected but only one layer enabled %d\n",i);
        return false;
    }
    this->trace.msg("AIMC: two step double weight computation selected, using layers %d and %d\n",i,k);
    for(int j=0;j<n_sectors&&sectors[j]!=-1;j++){
        
        this->layers[j][0]=i;
        this->layers[j][1]=k;
        this->layers[j][2]=-1;
    }
    return true;
}


void Pcm::enabled_sectors(uint32_t configuration){
    
    uint32_t config_mask = 0x00F00000;
    this->used_sectors = 0;

    this->trace.msg("AICM: sector enable: \n");

    int i=0;
    int j=0;

    for(j=0;j<n_sectors;j++){
        if((configuration&config_mask) != 0){
            this->sectors[i++] = j;
            this->trace.msg(" sector[%d]\n",j);
            used_sectors++;
        }
    
        config_mask = config_mask >> 4;
    }
    this->sectors[i] = -1; // End of sector marker
    
    this->negative_mask =(~0ULL << (this->cell_size*used_sectors*this->cells_per_weight));
    this->negative = 1 << ((this->cell_size*used_sectors*this->cells_per_weight)-1); 
    this->max_shift=(used_sectors*this->cells_per_weight*cell_size )-cell_size;
  
}

void Pcm::free_component(){
    delete[] this->input_vector;
    delete[] this->pcm_cells;
    delete[] this->mvm_full_result;
    delete[] this->aimc_response;
    delete[] this->sectors;
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