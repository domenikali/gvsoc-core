import gvsoc.systree

class Pcm(gvsoc.systree.Component):
    """Phase Changing Memory (PCM)

    This models a PCM memory module capable of Analog In-Memory Computing (AIMC), this module can be used solely as solid-state memory
    The PCM cells can be preloaded with initial data.
    It contains a timing model of a bandwidth, reported through latency.

    Attributes
    ----------
    parent: gvsoc.systree.Component
        The parent component where this one should be instantiated.
    name: str
        The name of the component within the parent space.
    
    Xi_size: int
        The size of each Xi values in bits. It's considered signed
    cell_size: int
        The size of the PCM cells in bits. It's considered signed
    output_size: int
        The size of each output value in bits (for maximal accuracy the output values should be equal to or greater than: Xi_size + cell_size -1)
    cells_per_weight
        The number of PCM cells used for each weigth
    
    tile_size: int
        the length of the tile in cells. The tile is the basic unit of computation.
    array_size: int
        the number of tiles for each sector
    n_sectors: int
        the number of sectors in the module

    pcm_width_log2: int 
        The log2 of the bandwidth to the PCMs, i.e. the number of bytes the PCM module can transfer per cycle. (for ease of use this value should be a multiple of cell_size)
    input_width_log2: int
        The log2 of the bandwidth of inputs values, i.e. the number of bytes for the Xi inputs bus transfer per cycle. (for ease of use this value should be a multiple of Xi_size)
    output_width_log2: int
        The log2 of the bandwidth of output values, i.e, the number of bytes for the output bus transfer per cycle. (for ease of use this value should be a multiple of output_size)
    
    aimc_bus_freq: int
        The frequency of the AIMC bus in MHz. This is the bus used to transfer data to and from the PCM cells.
    aimc_latency: int
        The latency of the AIMC bus in cycles. This is the time it takes to compute the MVM and perform the ADC.

    pcm_read_latency: int
        The latency of the PCM read operation in cycles. This is the time it takes to read data from the PCM cells.
    pcm_write_latency: int
        The latency of the PCM write operation in cycles. This is the time it takes to write data to the PCM cells.

        
    stim_file: str
        The path to a binary file which should be preloaded at the beginning of the memory. The format
        is a raw binary, and is loaded with and fread.
    power_trigger: bool
        True if the memory should trigger power report generation based on dedicated accesses.

    latency: int
        Specify extra latency which will be added to any incoming request.
    """
    def __init__(self, 
                 parent: gvsoc.systree.Component, 
                 name: str, 
                 Xi_size: int=8, 
                 cell_size: int=4,
                 output_size: int=11,
                 cells_per_weight: int=2,
                 tile_size:int =128,
                 array_size:int=4,
                 n_sectors:int=4,
                 pcm_width_log2: int=64,
                 input_width_log2: int=64,
                 output_width_log2: int=88,
                 width_log2: int=2,
                 aimc_bus_freq: int=500000,
                 aimc_latency: int=0,
                 pcm_read_latency: int=0,
                 pcm_write_latency: int=0,
                 stim_file: str=None, 
                 power_trigger: bool=False,
                 latency=0):

        super().__init__(parent, name)

        self.add_sources(['memory/pcm_mem.cpp'])


        self.add_properties({
            'Xi_size': Xi_size,
            'cell_size': cell_size,
            'output_size': output_size,
            'cells_per_weight' : cells_per_weight,
            'tile_size':tile_size,
            'array_size':array_size,
            'n_sectors':n_sectors,
            'pcm_width_log2': pcm_width_log2,
            'input_width_log2': input_width_log2,
            'output_width_log2': output_width_log2,
            'width_log2': width_log2,
            'aimc_bus_freq':aimc_bus_freq,
            'aimc_latency': aimc_latency,
            'stim_file': stim_file,
            'power_trigger': power_trigger,
            'latency': latency,
        })

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        """Returns the input port.

        Incoming requests to be handled by the memory should be sent to this port.\n
        It instantiates a port of type vp::IoSlave.\n

        Returns
        ----------
        gvsoc.systree.SlaveItf
            The slave interface
        """
        return gvsoc.systree.SlaveItf(self, 'input', signature='io')
