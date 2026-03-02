import sys
import os

# Ensure the current directory is in sys.path so we can import the .pyd file
sys.path.append(os.getcwd())

try:
    import PF_py
    print("Successfully imported PF_py")
    
    # Test ObsData
    obs = PF_py.ObsData()
    obs.x = 100.0
    obs.y = 200.0
    print(f"ObsData: x={obs.x}, y={obs.y}")
    
    # Test ParticleFilter
    pf = PF_py.ParticleFilter(100)
    print("ParticleFilter created with 100 particles")
    
    # Test initialization
    pf.initialize(obs, 1000.0, 5000.0, 10.0, 30.0, 0.0, 6.28, 0.1)
    print(f"Initialized: {pf.isInitialized()}")
    
    # Test prediction
    pf.predict(1.0, 1.0, 0.1)
    print("Prediction step done")
    
    # Test update
    pf.update(obs, 0.1)
    print("Update step done")
    
    # Test estimate
    est = pf.getEstimate()
    print(f"Estimate: {est}")
    
except ImportError as e:
    print(f"Failed to import PF_py: {e}")
except Exception as e:
    print(f"An error occurred: {e}")
