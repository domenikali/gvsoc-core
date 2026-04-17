"""

"""

import gvsoc.systree as st

class Mailbox(st.Component):
    """
    Mailbox

    This instantiates a mailbox component.

    Attributes
    ----------
    size : int
        Size of the mailbox (default: 0x1000).

    """

    def __init__(self, parent, name, size=None):
        super(Mailbox, self).__init__(parent, name)

        if size is None:
            size = 0x1000

        # Register all parameters as properties so that they can be overwritten from the command-line
        self.add_property('size', size)

        self.set_component('devices.mailbox')

    def i_INPUT(self) -> st.SlaveItf:
        """Returns the input port.

        Incoming requests to be handled by the memory should be sent to this port.\n
        It instantiates a port of type vp::IoSlave.\n

        Returns
        ----------
        gvsoc.systree.SlaveItf
            The slave interface
        """
        return st.SlaveItf(self, 'input', signature='io')
