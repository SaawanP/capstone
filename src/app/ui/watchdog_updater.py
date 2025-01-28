import os
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class UiFileHandler(FileSystemEventHandler):
    def __init__(self, ui_file, output_file):
        self.ui_file = ui_file
        self.output_file = output_file
    # def on_any_event(self, event):
    #     print(f"Event detected: {event.event_type} at {event.src_path}")


    def on_closed(self, event):
        # Skip if the modified file is the output file
        if event.src_path == self.output_file:
            return
        
        print(f"Detected modification: {event.src_path}")

        if event.src_path == self.ui_file:
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
    ui_file = os.path.expanduser("~/test/ui/new_interface.ui")
    output_file = os.path.expanduser("~/test/ui/new_interface.py")

    event_handler = UiFileHandler(ui_file, output_file)
    observer = Observer()
    observer.schedule(event_handler, path=os.path.expanduser("~/test/ui/"), recursive=False)

    print("Starting the observer...")
    observer.start()

    print(f"Watching for changes in {ui_file}...")
    try:
        observer.join()  # Wait for the observer thread to complete
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
