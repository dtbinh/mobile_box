import Tkinter as tk
import csv
import sys
from PIL import Image, ImageTk, ImageDraw

# Load Image

if len(sys.args>1):
        source = sys.args[1]
    if len(sys.args) > 2
        destination = sys.args[2]


class Map_To_Csv:
    def __init__(self, map_filename):
        self.rotate = 0
        self.map_filename = map_filename

        self.top = 0
        self.bottom = 100
        self.left = 0
        self.right = 100
        self.grid_row_ct = 3
        self.grid_col_ct = 3   
        self.grid = []
        
        
    def update_canvas(self):
        
        self.grid = []
        
        # clear screen
        self.canvas.delete(tk.ALL)
        
        # Draw image
        self.canvas.create_image(0,0,image = self.photo, anchor="nw")
        
        # Draw grid
        for row_id in range(self.grid_row_ct):
            self.grid.append([])
            for col_id in range(self.grid_col_ct):
                fill = None
                cell_top = row_id * (self.bottom-self.top) / self.grid_row_ct + self.top
                cell_left = col_id * (self.right-self.left) / self.grid_col_ct + self.left
                cell_bottom = (row_id + 1) * (self.bottom-self.top) / self.grid_row_ct + self.top
                cell_right = (col_id + 1) * (self.right-self.left) / self.grid_col_ct + self.left
                for im_row in range(cell_top, cell_bottom):
                    for im_col in range(cell_left, cell_right):
                        if fill == None:
                            if self.image.getpixel((im_col, im_row)) == 0:
                                fill = 'black'
                if fill == None:
                    self.grid[-1].append(0)
                else:
                    self.grid[-1].append(1)
                self.canvas.create_rectangle(
                    cell_left,
                    cell_top,
                    cell_right,
                    cell_bottom,
                    outline = "blue",
                    fill = fill
                )
        
    def set_top_left(self, event):
        self.top = event.y
        self.left = event.x
    
    def set_bottom_right(self, event):
        self.bottom = event.y
        self.right = event.x
        self.update_canvas()
        
    def move_grid(self, event):
        ver_dif = self.bottom - self.top
        hor_dif = self.right - self.left
        self.top = event.y
        self.left = event.x
        self.bottom = self.top + ver_dif
        self.right = self.left + hor_dif
        self.update_canvas()
        
    def save_to_file(self, event = None):
        with open(self.entry.get() + '.csv', 'w') as file:
            writer = csv.writer(file, delimiter = ",", lineterminator='\n')
            for row in self.grid:
                writer.writerow(row)
        draw_image = self.image.copy()
        draw = ImageDraw.Draw(draw_image)
        for row_id in range(self.grid_row_ct):
            for col_id in range(self.grid_col_ct):
                
                cell_top = row_id * (self.bottom-self.top) / self.grid_row_ct + self.top
                cell_left = col_id * (self.right-self.left) / self.grid_col_ct + self.left
                cell_bottom = (row_id + 1) * (self.bottom-self.top) / self.grid_row_ct + self.top
                cell_right = (col_id + 1) * (self.right-self.left) / self.grid_col_ct + self.left
                fill = None
                if self.grid[row_id][col_id] == 1:
                    fill = 0
                draw.rectangle(
                    [cell_left, cell_top, cell_right, cell_bottom],
                    outline = 0,
                    fill = fill
                )
        del draw

        # write to stdout
        draw_image.save(self.entry.get() + ".pgm", "PPM")
        self.dialog.destroy()        
                
    
    def save_dialog(self, event):
        # Create Dialog Box
        self.dialog = tk.Toplevel(self.app)
        label = tk.Label(self.dialog, text="Filename")
        label.pack()
        self.entry = tk.Entry(self.dialog)
        self.entry.pack(padx=5)
        self.entry.focus_set()
        button = tk.Button(self.dialog, text="OK", command=self.save_to_file)
        button.pack(pady=5)

        self.dialog.bind("<Return>", self.save_to_file)
        
        
        
    def increment_row(self, event):
        self.grid_row_ct += 1
        self.update_canvas()
        
    def decrement_row(self, event):
        self.grid_row_ct -= 1
        if self.grid_row_ct < 3:
            self.grid_row_ct = 3
        self.update_canvas()
    
    def increment_col(self, event):
        self.grid_col_ct += 1
        self.update_canvas()
        
    def decrement_col(self, event):
        self.grid_col_ct -= 1
        if self.grid_col_ct < 3:
            self.grid_col_ct = 3
        self.update_canvas()
    
    def rotate_left(self, event):
        self.rotate -= 5
        self.rotate += 360
        self.rotate %= 360
        
        self.image = self.source_image.rotate(self.rotate)
        self.photo = ImageTk.PhotoImage(master = self.app, image = self.image)
        self.update_canvas()
        
    def rotate_right(self,event):
        self.rotate += 4
        self.rotate += 360
        self.rotate %= 360

        self.image = self.source_image.rotate(self.rotate)
        self.photo = ImageTk.PhotoImage(master = self.app, image = self.image)
        self.update_canvas()
    
    def run(self):
        self.app = tk.Tk()
        
        self.canvas = tk.Canvas(self.app, width=600, height=600)
        self.canvas.pack()
        
        self.source_image = Image.open(self.map_filename)
        self.image = self.source_image.rotate(self.rotate)
        self.photo = ImageTk.PhotoImage(master = self.app, image = self.image)
        
        self.update_canvas()
        
        self.app.bind("<Down>", self.decrement_row)
        self.app.bind("<Up>", self.increment_row)
        self.app.bind("<Left>", self.decrement_col)
        self.app.bind("<Right>", self.increment_col)
        self.app.bind("<space>", self.save_dialog)
        self.app.bind("z" , self.rotate_left)
        self.app.bind("x", self.rotate_right)
        self.canvas.bind("<Button-1>", self.set_top_left)
        self.canvas.bind("<Button-3>", self.move_grid)
        self.canvas.bind("<B3-Motion>", self.move_grid)
        self.canvas.bind("<B1-Motion>", self.set_bottom_right)
        
        print ("Controls: \n\tArrowKeys\n\tx,z\n\tleft,right mouse (drag)\n")
        print ("Space to save")
        
        
        self.app.mainloop()
        
        
mtc = Map_To_Csv("MapDAILab.pgm")
mtc.run()

#display map using tkinter

#describe a transformation

#save the new map with the transformation and grid lines