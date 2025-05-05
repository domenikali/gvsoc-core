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

        //total size in bytes of the PCM module
        size_t pcm_size;

        //  Registers for AIMC: 
        //  - input registers (Xi values)
        uint32_t *input_registers;
        //  - AIMC configuration register (i.e. command, tiles and sectors, ...)
        uint32_t *configuration_registers;
        
        //weigth matrix, it's allocatated as a flat array but it's accesed as a 5D matrix
        int8_t * pcm_cells; 

        //AIMC responses helper:
        //  - number of bytes for each Yi output value
        int response_byte_size;
        //  - ready to send AIMC computation result as byte array
        uint8_t * aimc_response;
        //  - full AIMC result (before ADC and clipping)
        int64_t * mvm_full_result;
        //  - pending request for asynchronous response
        vp::IoReq *pending_req;



        //TODO: figure out how to map the PCM cells in a smart way (map? flat out? O(1) space c. would be better but I'm not sure if possible)
        int Xi_adresses;// equal to max_matrix_y / input_width_log2
        int pcm_cells_adresses;// equal to max_matrix_y / pcm_width_log2   

        //MVM and ADC logic

        /**
         * @brief 2D array with enabled sectors
         * This method returns a 2D array with the enabled sectors for each column
         * @param configuration_registers pointer to the configuration registers
         * @return pointer to the 2D array with the enabled sectors
         * @note the output array is allocated in the heap and must be freed by the caller
         */
        int ** enabled_sectors(uint32_t *configuration_registers);

        /**
         * @brief MVM tile portion computation
         * This method compute a portion of the MVM operation, it is called by the main MVM method and run on a separete thread
         * @param matrix pointer to the PCM cells
         * @param vector pointer to the input vector
         * @param result pointer to the output vector
         * @param sector pointer to the sector array (describes whitch sector/s are active)
         * @param tile tile number
         * @param i index of the tile
         * @param inc increment value
         */
        void compute_flat_mvm_tile(int8_t *matrix, int8_t * vector, int64_t * result, int *sector, int tile, int i,int inc);

        /**
         * @brief MVM multithreaded method
         * This method compute the MVM operation using multiple threads, each thread compute a portion of the MVM operation
         * @param matrix pointer to the PCM cells
         * @param vector pointer to the input vector
         * @param sector pointer to the sector array (describes whitch sector/s are active for each tile array)
         * @param result pointer to the output vector
         */
        void mvm_multithreaded(int8_t* matrix, int8_t * vector, int **sector,int64_t * result);

        /**
         * @brief ADC method
         * This method convert the input value into a byte array cutting bit's over the max voltage, it is used to convert the output value of the MVM operation into a byte array ready to be returned via IoReq
         * @param input input value to be converted
         * @param unsigned_conversion if true the conversion is unsigned, otherwise it is signed
         * @return pointer to the byte array
         * @note the output array is allocated in the heap and must be freed by the caller
         * @note the output array is not aligned to 8 bits, the first byte is the most significant
         */
        uint8_t * Pcm::adc(int64_t input,bool unsigned_conversion);

};