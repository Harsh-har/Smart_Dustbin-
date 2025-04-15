import time
from l21 import GarbageProcessor
from l22 import SortingMechanism



if __name__ == "__main__":
    garbage_processor = GarbageProcessor()
    sorting_mechanism = SortingMechanism()

    print("\n🔄 Starting Garbage Processing...")
    garbage_processor.process_garbage()  # Execute first layer (polythene detection & processing)

    print("\n⏳ Waiting for first layer to finish...")
    time.sleep(3)  # Optional delay

    print("\n🔄 Starting Sorting Mechanism...")
    sorting_mechanism.process_sorting()  # Execute second and third layers (sorting & rotation)