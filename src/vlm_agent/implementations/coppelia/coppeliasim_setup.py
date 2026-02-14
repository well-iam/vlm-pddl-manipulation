import logging
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

logger = logging.getLogger(__name__)

def connect_to_sim(host='localhost', port=23000):
    """Establishes connection with CoppeliaSim and sets synchronous mode."""
    logger.debug("Attempting connection to CoppeliaSim...")
    client = RemoteAPIClient(host, port)
    # client.setStepping(True)
    try:
        sim = client.require('sim')
        simIK = client.require('simIK')
        simOMPL = client.require('simOMPL')
        logger.info("✅ Connection to CoppeliaSim and IK/OMPL modules established.")
        return client, sim, simIK, simOMPL
    except Exception as e:
        logger.exception("❌ Connection failed:")
        return None, None, None, None