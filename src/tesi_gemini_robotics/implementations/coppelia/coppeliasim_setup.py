import logging
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

logger = logging.getLogger(__name__)

def connect_to_sim(host='localhost', port=23000):
    """Stabilisce la connessione con CoppeliaSim e imposta la modalità sincrona."""
    logger.debug("Tentativo di connessione a CoppeliaSim...")
    client = RemoteAPIClient(host, port)
    # client.setStepping(True)
    try:
        sim = client.require('sim')
        simIK = client.require('simIK')
        simOMPL = client.require('simOMPL')
        logger.info("✅ Connessione a CoppeliaSim e ai moduli IK/OMPL stabilita.")
        return client, sim, simIK, simOMPL
    except Exception as e:
        logger.exception("❌ Connessione fallita:")
        return None, None, None, None