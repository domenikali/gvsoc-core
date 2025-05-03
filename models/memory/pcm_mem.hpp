#pragma once

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>
#include <stdio.h>
#include <string.h>

/**
 * @brief PCM module with AIMC capabilities
 * This class models the behaviour of a PCM module with AIMC capabilities, this module is composed of multiple PCM tiles divided into arrays.
 * In order to access this module there are two buses, one to access PMC arrays to load and store values (the PMC module can be used solely as a non-volatile memory),
 * and a second bus to use all the AIMC capabilities, using this bus an input array, divided into Xi values, can be loaded to the module. After the computation a Yi output array is returned.
 * Does this operation use a cache as a buffer for both reading and writing?
 * @author Leonardo Domenicali, UniBo (leonardo.domenicali@gmail.com | leonardo.domenicali@studio.unibo.it)
 */
class Pcm : public vp::Component
{
    public:
        Pcm(vp::ComponentConf &config);

        void reset (bool active);
        /**
         * @brief static PCM related call
         * This method allow for load and store from and to the PCMs cells
         */
        static vp::IoReqStatus req_PCM(vp::Block *__this, vp::IoReq *req);
        /**
         * @brief static AIMC related call
         * This method allow to store Xi values for AIMC and load Yi output values after AIMC is complete
         */
        static vp::IoReqStatus req_AIMC(vp::Block *__this, vp::IoReq *req);

        void free_component();

    private:

        vp::ClockEvent event;

        static void power_ctrl_sync(vp::Block *__this,bool value);


        //Read and write operations to/from PCM
        vp::IoReqStatus handle_PCM_write(uint64_t addr, uint64_t size, uint8_t *data);
        vp::IoReqStatus handle_PCM_read(uint64_t addr, uint64_t size, uint8_t *data);

        //AIMC Xi write and Yi read
        vp::IoReqStatus handle_Xi_write(vp::IoReq *req);

        //AIMC computations start with custom command
        vp::IoReqStatus handle_AIMC_compute(vp::IoReq *req);

        //timed aimc response
        static void aimc_computation(vp::Block* __this, vp::ClockEvent *event);


        //Analog to digital conversion, cut of out of range bits and move into usigned bytes (if not all bits are used most important are going to be 0)
        uint8_t * adc(int64_t value);
        
        //slave port for PCM related operations
        vp::IoSlave input_PCM;
        //slave port for AIMC related operations
        vp::IoSlave input_AIMC;


        //module powered (true powered, fase unpowered)
        bool powered_up;
        vp::WireSlave<bool> power_ctrl_itf;

        vp::Trace trace;

        //architectural parameters
        int Xi_size;
        int cell_size;
        int output_size;
        int cells_per_weight;

        int tile_size;
        int array_size;
        int n_sectors;
        int matrix_length;

        int pcm_width_log2;
        int input_width_log2;
        int output_width_log2;

        uint64_t aimc_bus_freq;
        int aimc_latency;

        int pcm_read_latency;
        int pcm_write_latency;

        size_t pcm_size;

        uint32_t *input_registers;
        uint32_t *configuration_registers;

        
        //weigth matrix, it's allocatated as a flat array but it's accesed as a 5D matrix
        int8_t * pcm_cells; 

        uint8_t * aimc_response;
        vp::IoReq *pending_req;



        //TODO: figure out how to map the PCM cells in a smart way (map? flat out? O(1) space c. would be better but I'm not sure if possible)
        int Xi_adresses;// equal to max_matrix_y / input_width_log2
        int pcm_cells_adresses;// equal to max_matrix_y / pcm_width_log2   
};


