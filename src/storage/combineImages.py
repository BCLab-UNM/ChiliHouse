#!/usr/bin/env python3
import os
import sys
from PIL import Image, ImageDraw, ImageFont, UnidentifiedImageError
import glob

def combine_images(folder):
    # Get all the files matching each pattern
    rgb_files = sorted(glob.glob(os.path.join(folder, 'RGB_*.png')))
    ir_files = sorted(glob.glob(os.path.join(folder, 'IR_*.png')))
    depth_files = sorted(glob.glob(os.path.join(folder, 'Depth_*.png')))

    # Define the maximum count of images to define loop range
    max_length = max(len(rgb_files), len(ir_files), len(depth_files))
    
    # Specify the font (update the path to your font file)
    font_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyretermes-regular.otf'
    font_size = 48  # Set the font size
    font = ImageFont.truetype(font_path, font_size)
    
    # Use index-based loop to handle different lengths
    for i in range(max_length):
        images = []
        for file_list in (rgb_files, ir_files, depth_files):
            img = None
            try:
                img = Image.open(file_list[i])
            except IndexError:
                # If there is no corresponding file, create a black placeholder
                if images:  # Check if there's already an image to match size
                    width, height = images[0].size
                    img = Image.new('RGB', (width, height), "black")
                else:
                    img = Image.new('RGB', (640, 480), "black")  # Default size, change if needed
            except UnidentifiedImageError:
                pass
            if img:
                images.append(img)

        # Combine the images
        widths, heights = zip(*(i.size for i in images))
        total_width = sum(widths)
        max_height = max(heights)
        new_im = Image.new('RGB', (total_width, max_height))

        x_offset = 0
        for im in images:
            try:
                new_im.paste(im, (x_offset, 0))
                x_offset += im.size[0]
            except:
                pass

        # Add timestamp to the bottom right of the combined image
        draw = ImageDraw.Draw(new_im)
        timestamp = os.path.basename(rgb_files[i]).replace('RGB_', '').replace('.png', '') if i < len(rgb_files) else f'missing_{i+1}'
        text_width, text_height = draw.textsize(timestamp, font=font)
        draw.text((total_width - text_width - 10, max_height - text_height - 10), timestamp, fill=(255, 255, 255), font=font)

        # Save the new image with a new filename in the current working directory
        base_name = f'Combined_{timestamp}.png'
        new_im.save(base_name)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: ./combine_images.py <folder>")
        sys.exit(1)

    folder_path = sys.argv[1]
    combine_images(folder_path)
