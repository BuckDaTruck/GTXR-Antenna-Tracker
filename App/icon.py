import tkinter as tk
from PIL import Image, ImageTk

def set_app_icon(window, icon_path):
    img = Image.open(icon_path)
    img = ImageTk.PhotoImage(img)
    window.tk.call('wm', 'iconphoto', window._w, img)

def main():
    # Create main window
    root = tk.Tk()
    root.title("App with Icon")

    # Set the icon for the app
    icon_path = "App/icon.ico"
    set_app_icon(root, icon_path)


    root.mainloop()

if __name__ == "__main__":
    main()
