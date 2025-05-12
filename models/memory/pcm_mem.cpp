#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>
#include <stdio.h>

#include "pcm_mem.hpp"

Pcm::Pcm(vp::ComponentConf &config) : vp::Component(config) , event( this, Pcm::aimc_computation){
    
    traces.new_trace("trace", &trace, vp::DEBUG);

    input_PCM.set_req_meth(&Pcm::req_PCM);
    new_slave_port("intput",&input_PCM);

    input_AIMC.set_req_meth(&Pcm::req_AIMC);
    new_slave_port("input",&input_AIMC);

    this->power_ctrl_itf.set_sync_meth(&Pcm::power_ctrl_sync);
    new_slave_port("power_ctrl",&this->power_ctrl_itf);

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

    this->pcm_width_log2=this->get_js_config()->get("pcm_width_log2")->get_int();
    this->input_width_log2=this->get_js_config()->get("input_width_log2")->get_int();
    this->output_width_log2=this->get_js_config()->get("output_width_log2")->get_uint();

    this->aimc_bus_freq = this->get_js_config()->get("aimc_bus_freq")->get_int();
    this->aimc_latency=this->get_js_config()->get("aimc_latency")->get_int();
    this->pcm_read_latency=this->get_js_config()->get("pcm_read_latency")->get_int();
    this->pcm_write_latency=this->get_js_config()->get("pcm_write_latency")->get_int();

    this->pcm_size = this->cells_per_weight*this->tile_size*this->array_size*this->n_sectors*this->matrix_length;
    this->mvm_full_result = new int64_t[this->matrix_length];

    //number of bytes for each Yi value in the output vector (tells how many bytes are needed to store the output value using all it's aviable bits)
    this->response_byte_size = this->output_size%(sizeof(uint8_t))+1;
    this->response_byte_size+=((this->output_size - this->response_byte_size)/(sizeof(uint8_t)*8));

    //number of Xi adresses in bit is equal to the number of Xi values deided by the width of the input bus
    this->Xi_adresses = matrix_length/input_width_log2;
    //number of pcm cells adresses in bit is equal to the number of cells devided by the width of the pcm bus
    this->pcm_cells_adresses = (matrix_length*matrix_length)/pcm_width_log2;


    trace.msg("Builing PCM input vector (Size: 0x%x)\n",this->matrix_length);
    //input value array allocation, 
    this->input_registers = new uint32_t[(this->matrix_length* this->Xi_size)/(32-(32%this->Xi_size))];//this formula is used to allocate the register input bufffer in 32bits spaces, the Xi_size can range from 1 to 32, worst case 1 register is associated with 1 input value (in cases like Xi_size > 16)
    if(this->input_registers==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: input vector allocation failed\n");
        throw std::bad_alloc();
    }
    //memset(this->input_registers, 0,); do i need this?

    trace.msg("Builing PCM Memory (Size: 0x%x)\n",this->pcm_size);
    //matrix allocation
    this->pcm_cells = new int8_t[this->pcm_size];
    if(this->pcm_cells==nullptr){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: matrix allocation failed\n");
        throw std::bad_alloc();
    }
    //if memory is small, fill it with a special value to detect uninitialized variables
    if (this->pcm_size < (2<<24))
    {
        memset(this->pcm_cells, 0, this->pcm_size);    
    }

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

vp::IoReqStatus Pcm::req_PCM(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;

    if(req->get_is_write()){
        req->inc_latency(_this->pcm_write_latency);
        return _this->handle_PCM_write(req->get_addr(), req->get_size(),req->get_data());
    }
    else{
        req->inc_latency(_this->pcm_read_latency);
        return _this->handle_PCM_read(req->get_addr(), req->get_size(),req->get_data());
    }

}

vp::IoReqStatus Pcm::handle_PCM_read(uint64_t addr, uint64_t size, uint8_t *data){
    if(addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: read out of bounds\n");
        return vp::IO_REQ_INVALID;
    }
    //read the PCM value from the memory
    memcpy((void *)data, (void*)this->pcm_cells[addr], size);
    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_PCM_write(uint64_t addr, uint64_t size, uint8_t *data){
    if(addr>this->pcm_size){
        this->trace.msg(vp::Trace::LEVEL_ERROR,"PCM: write out of bounds\n");
        return vp::IO_REQ_INVALID;
    }
    //write the PCM value into the memory
    memcpy((void*)this->pcm_cells[addr], (void *)data, size);
    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::req_AIMC(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;

    if(req->get_is_write()){
        return _this->handle_Xi_write(req);
    }
    else{
        return _this->handle_AIMC_compute(req);
    }
}

vp::IoReqStatus Pcm::handle_Xi_write(vp::IoReq *req){
    uint64_t addr = req->get_addr();
    uint64_t size = req->get_size();
    uint8_t *data = req->get_data();

    req->inc_latency((uint64_t) (this->clock.get_engine()->get_period()/this->aimc_bus_freq) );
    //write the Xi value into the input register
    memcpy((void*)this->input_registers[addr], (void *)data, size);//this is not as easy if the size of Xi is aligned
    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_AIMC_compute(vp::IoReq *req){
    
    if(!this->event.is_enqueued()){
        this->trace.msg("AIMC computation enqued\n");
        this->pending_req = req;
        

        int ** sectors = this->enabled_sectors(this->configuration_registers);
        //this->mvm_multithreaded(this->pcm_cells, TODO: how to use vectors, sectors, this->mvm_full_result);
        this->convert_to_adc(this->mvm_full_result, this->aimc_response);
        

        return vp::IO_REQ_PENDING;

    }
    return vp::IO_REQ_INVALID;
}

void Pcm::aimc_computation(vp::Block* __this, vp::ClockEvent *event){
    Pcm *_this = (Pcm *)__this;
    _this->trace.msg("AIMC computation result returned\n");
    
    //*(uint8_t*)_this->pending_req->get_data() = _this->aimc_response;
    _this->pending_req->get_resp_port()->resp(_this->pending_req);
}


void Pcm::compute_flat_mvm_tile(int8_t *matrix, int8_t * vector, int64_t * result, int *sector, int tile, int i,int inc){
    for(int j=i;j<i+inc;j++){
        for(int k=0;k<this->matrix_length;k++){
            //create an all-zero int64_t
            int64_t weight=static_cast<int64_t>(0);
            uint8_t y=0;
            //create an indexing for the bit shift
            int bit_index=this->max_shift;
            
            while(sector[y]>=0){
                for(int x=0;x<this->cells_per_weight;x++){
                    //generate the index of the flat 5D array
                    uint64_t index = (((sector[y]*this->array_size)+i)*this->tile_size+j)*this->matrix_length+k*this->matrix_length+x;
                    //the mask is used to select the bits of the cell, then it's shifted to the right position
                    weight |= ((matrix[index]&this->mask)<<bit_index);
                    //the bit index is decremented by the size of the cell
                    bit_index-=this->cell_size;   
                }
                y++;
            }
            //check if the weight is negative
            if((weight&this->negative)!=0){
                //if the weight is negative make the 2's complement of it
                weight = static_cast<int64_t>(static_cast<uint64_t>(weight) | negative_mask);
            }
            //write the result into the result vector
            result[tile*this->tile_size+j] += weight * vector[k];
        }
    }
}

  
void Pcm::mvm_multithreaded(int8_t* matrix, int8_t * vector, int **sector,int64_t * result){
    
    //result vector initialization
    memset(result,0,sizeof(int64_t)*this->matrix_length);
  
    //threads vector
    std::vector<std::thread> threads;
    //loops, each thread will compute a part of a tile of the matrix
    for(int i=0;i<this->array_size;i++){
        for(int j=0;j<2;j++){
            std::thread t(compute_flat_mvm_tile, matrix, vector, result, sector[i], i,j*64, 64);
            threads.push_back(move(t));
        }
    }
  
    //thread join, not necesary but highly raccomended for extensive mvm use
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
      

    //output buffer initialization, loads the less significant byte first at the end of the array
    for (int i = 0; i < this->response_byte_size; ++i) {
        output[this->response_byte_size-i-1] = input >> (i * 8) & 0xFF;
    }  

    //clipping
    uint8_t mask=0xFF;
    int u_conversion = this->signed_conversion ? 0 : 1; 
    mask>>=(8-(this->output_size)+u_conversion)%8;

    //masking most significant byte
    output[0] &= mask;

}

int ** Pcm::enabled_sectors(uint32_t *configuration_registers){
    int ** sectors = new int*[this->array_size];
    for(int i=0;i<this->n_sectors;i++){
        sectors[i] = new int[this->n_sectors+1];
        int j=0;
        int k=0;
        do{
            //TODO: condition to configure the sectors 
            k++;
        }while (k!= 0);
    }

    this->used_sectors = 0;
    int i=0;
    while(sectors[0][i]>=0){
      this->used_sectors++;
      i++;
    }
    this->signed_conversion = true;
    this->negative_mask =(~0ULL << (this->cell_size*used_sectors*this->cells_per_weight));
    this->negative = 1 << ((this->cell_size*used_sectors*this->cells_per_weight)-1); 
    this->max_shift=(used_sectors*this->cells_per_weight*cell_size )-cell_size;
    return sectors;
}

void Pcm::free_component(){
    delete[] this->input_registers;
    delete[] this->pcm_cells;
    delete[] this->mvm_full_result;
    delete[] this->aimc_response;
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