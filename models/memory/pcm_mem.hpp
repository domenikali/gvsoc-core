#pragma once

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <vector>
#include <bitset>
#include <functional>
#include <atomic>

#define PCM_SIZE_8 1
#define INPUT_SIZE_8 1


#ifdef PCM_SIZE_8
    typedef uint8_t pcm_size_t;
#elif defined(PCM_SIZE_16)
    typedef uint16_t pcm_size_t ;
#elif defined(PCM_SIZE_32)
    typedef uint32_t pcm_size_t ;
#else 
    typedef uint64_t pcm_size_t;
#endif

#ifdef INPUT_SIZE_8
    typedef int8_t input_size_t;
#elif defined(INPUT_SIZE_16)
    typedef int16_t input_size_t;
#elif defined(INPUT_SIZE_32)
    typedef int32_t input_size_t;
#else 
    typedef int64_t input_size_t;
#endif

enum class CMD : uint32_t{
    CMD_MASK = 0xF0000000,
    CMD_COMPUTE = 0x00000000,
    CMD_PARAM = 0x10000000,
    CMD_ABORT = 0x40000000
};

enum class CMD_SETTINGS : uint32_t{
    CMD_SETTINGS_SECTORS = 0x10000000,
    CMD_SETTINGS_TWO_STEP_U = 0x11000000,
    CMD_SETTINGS_TWO_STEP_U_DOUBLE_WEIGHT = 0x12000000,
    CMD_SETTINGS_TWO_STEP_S = 0x13000000,
    CMD_SETTINGS_TWO_STEP_S_DOUBLE_WEIGHT = 0x14000000,
    CMD_SETTINGS_SINGLE_STEP = 0x15000000,
    CMD_SETTINGS_FAST_SINGLE_STEP = 0x16000000,
    CMD_SETTINGS_INPUT_PRECISION = 0x17000000,
    CMD_SETTINGS_BL = 0x18000000,
    CMD_SETTINGS_MASK = 0xFF000000
};

struct pcm_write_count{
    uint64_t reset_count;
    uint64_t write_count;
};
typedef struct pcm_write_count pcm_write_count_t;


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

        //void reset (bool active);

        /**
         * @brief PCM request handler
         * This method dispatches the request to the right method based on the address of the request.
         * the first 4 bytes are the AIMC cmd, the following bytes are for the AIMC Xi vector, the last bytes are for the PCM cells.
         * @note The PCM cells are not necessary for the module if the values are loaded from binary files and used as fixed memory.
         */
        static vp::IoReqStatus req_PCM_module(vp::Block *__this, vp::IoReq *req);   

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

        //          Read and write operations to/from PCM
        vp::IoReqStatus handle_PCM_write(vp::IoReq *req);
        
        vp::IoReqStatus handle_PCM_read(vp::IoReq *req);
        
        size_t pcm_output_size;

        //AIMC Xi write
        vp::IoReqStatus handle_Xi_write(vp::IoReq *req);

        //AIMC Yi read
        vp::IoReqStatus handle_Yi_read(vp::IoReq *req);

        //handles settings change with command
        vp::IoReqStatus aimc_settings(uint32_t cmd);

        vector<pid_t> aimc_threads;
        //abort AIMC operation
        void aimc_abort();

        //AIMC computations start with custom command
        vp::IoReqStatus handle_AIMC_compute(vp::IoReq *req);

        //timed aimc response
        static void aimc_computation(vp::Block* __this, vp::ClockEvent *event);
        
        //slave port for PCM related operations
        //vp::IoSlave input_PCM;
        //slave port for AIMC related operations
        //vp::IoSlave input_AIMC;

        //slave port for PCM & AIMC related operations, dispatch to the right method based on addr 
        vp::IoSlave input_PCM_module;

        //module powered (true powered, fase unpowered)
        bool powered_up;
        vp::WireSlave<bool> power_ctrl_itf;

        vp::Trace trace;

        //statistic parameters
        //  - number of read operations to the PCM module
        uint64_t pcm_read_count;
        //  - number of write operations to the PCM module
        //    - number of cell resets
        //    - number of cell writes 
        pcm_write_count_t pcm_write_count;
        //  - number of AIMC computations
        uint64_t aimc_compute_count;

        //architectural parameters
        int Xi_size;
        int cell_size;
        int output_size;
        int cells_per_weight;

        int tile_size;
        int array_size;
        int n_layers;
        int n_sectors;
        int matrix_length;

        int pcm_width_log2;
        int input_width_log2;
        int output_width_log2;

        uint64_t aimc_bus_freq;
        int aimc_latency;

        int pcm_read_latency;
        int pcm_write_latency;

        int mvm_threads;

        //total size in bytes of the PCM module
        size_t pcm_size;

    
        // AIMC configuration register (i.e. command, tiles and sectors, ...)
        uint32_t configuration_registers;
        
        // matrix identifying used sectors during computation
        int8_t* sectors;
        // layer for each sectors to use
        int8_t** layers;
        //weigth matrix, it's allocatated as a flat array but it's accesed as a 5D matrix
        pcm_size_t * pcm_cells; 

        // Xi array
        input_size_t * input_vector;

        //AIMC responses helper:
        //  - number of bytes for each Yi output value
        int response_byte_size;
        //  - ready to send AIMC computation result as byte array
        uint8_t * aimc_response;
        //  - full AIMC result (before ADC and clipping)
        int64_t * mvm_full_result;
        //  - pending request for asynchronous response
        vp::IoReq *pending_req;

        //AIMC helper parameters
        //  - number of sectors active during the AIMC operation
        int used_sectors;
        //  - mask to check negative number
        uint64_t negative_mask;
        //  - mask to 2's complement the negative number
        uint64_t negative;
        //  - number of shift to apply to each cell to make the weight 
        int max_shift;
        //  - mask to get the used bits of the cell
        uint8_t mask;
        //  - true if the computation required signed multiplication or not
        bool signed_computation;
        //  - mask to apply to the input vector based on the input precision
        input_size_t precision_mask;

        //TODO: figure out how to map the PCM cells in a smart way (map? flat out? O(1) space c. would be better but I'm not sure if possible)
        int Xi_adresses;// equal to max_matrix_y / input_width_log2
        int pcm_cells_adresses;// equal to max_matrix_y / pcm_width_log2   

        //MVM and ADC logic

        /**
         * @brief Select enabled sectors
         * This method set an array with the enabled sectors 
         * @param configuratio configuration cmd, 32 bits unsigned integer
         */
        void enabled_sectors(uint32_t configuration);

        /**
         * @brief Select enabled layers
         * This method set a 2D array with the enabled layers for each sector
         * @param configuration configuration cmd, 32 bits unsigned integer
         * @param two_setp true if the two step mode is enabled, false otherwise
         * @return true if the configuration is correct, false otherwise
         * @note in single step mode only the configuration register is used to set the layer and the first one aviable is used for all the sectors
        */
        bool enabled_layers(uint32_t configuration,bool two_setp);

        /**
         * @brief set input precision
         * This method set the input precision mask based on the configuration cmd
         * @param cmd configuration cmd, 32 bits unsigned integer
         */
        void set_precision(uint32_t cmd);

        /**
         * @breif flat 4D array index computation
         * This method computes the index of the flat 4D array using the given parameters
         * @param s sector number
         * @param l layer number
         * @param y row number
         * @param x column number
         * @return index of the flat 4D array
         */
        inline long long inedx(int s,int l,int y,int x){
            return (((s*n_layers+l)*tile_size+y)*matrix_length)+x;
        }

        /**
         * @brief MVM single threaded method
         * This method compute the MVM operation using a single thread, it is the best suited for speed when the compiler optimizations are enabled
         * @param matrix pointer to the PCM cells
         * @param vector pointer to the input vector
         * @param layers pointer to the layer array (describes whitch layer/s are active for each sector)
         * @param sectors pointer to the sector array (describes whitch sector/s are active for each tile array)
         * @param result pointer to the output vector
         * 
         */
        void optimised_singlethreaded(pcm_size_t* matrix, input_size_t* vector,  int8_t**  layers, int8_t* sectors, int64_t* result);


        /**
         * @brief MVM multithreaded method
         * This method compute the MVM operation using multiple threads, each thread compute a portion of the MVM operation
         * @param matrix pointer to the PCM cells
         * @param vector pointer to the input vector
         * @param layers pointer to the layer array (describes whitch layer/s are active for each sector)
         * @param sectors pointer to the sector array (describes whitch sector/s are active for each tile array)
         * @param result pointer to the output vector
         * @note number of threads can be changed by the user through the pcm_n_threads property within the python generator
         * @note unfortunately this MVM require atomic operations in order to avoid race conditions
         */
        void optimised_multithreaded(pcm_size_t* matrix, input_size_t* vector,  int8_t** layers, int8_t* sectors, int64_t* result);

        /**
         * @brief MVM worker thread method
         * This method compute a portion of the MVM operation, it is called by the main MVM method and run on a separete thread
         * @param matrix pointer to the PCM cells
         * @param sector sector number
         * @param vector pointer to the input vector
         * @param result pointer to the output vector
         * @param y_start start index of the output vector
         * @param end end index of the output vector
         * @param matrix_length length of the matrix
         */
        static void multithreaded_worker_thread( pcm_size_t* matrix, int8_t sector,input_size_t* vector, std::atomic<int64_t>* result,int y_start,int end,int matrix_length);



        /**
         * @brief MVM method
         * This method compute the MVM using the best suited algorithm based on the configuration, requested multithreading and compiler optimisation
         * @param matrix pointer to the PCM cells
         * @param vector pointer to the input vector
         * @param layers pointer to the layer array (describes whitch layer/s are active for each sector)
         * @param sectors pointer to the sector array (describes whitch sector/s are active for each tile array)
         * @param result pointer to the output vector
         * @note this method is a dispatcher to the best suited MVM implementation, use compiler flags to enable the specific algorithm wanted from higlevel
         */
        void mvm(pcm_size_t*matrix,input_size_t*vector, int8_t** layers, int8_t*sectors,int64_t*result);

        /**
         * @brief Convert the MVM full sized result to bite array
         * This method convert the MVM full int 64 bits result to a unsigned byte array, it is used to convert the output value of the MVM operation into a byte array ready to be returned via IoReq
         * @param mvm_full_result pointer to the MVM result
         * @param aimc_response pointer to the AIMC response
         */
        void adc(int64_t * mvm_full_result, uint8_t * aimc_response);

        

};