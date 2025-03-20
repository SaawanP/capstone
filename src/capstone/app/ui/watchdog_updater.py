import os
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class UiFileHandler(FileSystemEventHandler):
    def __init__(self, ui_file, output_file):
        # Store absolute paths for consistent comparisons
        self.ui_file = os.path.abspath(ui_file)
        self.output_file = os.path.abspath(output_file)

    def on_modified(self, event):
        if event.is_directory:
            return
        
        basename = os.path.basename(event.src_path)
        # Trigger regeneration if the event is from the actual UI file
        # or from a temporary file (name starting with "#")
        if basename == os.path.basename(self.ui_file) or basename.startswith("#"):
            print(f"Detected modification: {event.src_path}")
            print(f"{self.ui_file} was modified. Regenerating {self.output_file}...")
            try:
                subprocess.run(
                    ["pyside6-uic", self.ui_file, "-o", self.output_file],
                    check=True,
                )
                print(f"Successfully regenerated {self.output_file}.")
            except subprocess.CalledProcessError as e:
                print(f"Error regenerating {self.output_file}: {e}")

if __name__ == "__main__":
    ui_file = "/home/vs9/fydp/src/capstone/app/ui/new_interface.ui"
    output_file = "/home/vs9/fydp/src/capstone/app/ui/new_interface.py"

    event_handler = UiFileHandler(ui_file, output_file)
    observer = Observer()
    observer.schedule(event_handler, path="/home/vs9/fydp/src/capstone/app/ui/", recursive=False)

    print("Starting the observer...")
    observer.start()
    print(f"Watching for changes in {ui_file}...")

    try:
        observer.join()  # Wait for the observer thread to complete
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
