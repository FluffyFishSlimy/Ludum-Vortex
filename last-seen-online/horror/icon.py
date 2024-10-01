from PIL import Image

def resize_image(image_path, size):
    img = Image.open(image_path)
    img = img.resize(size, Image.NEAREST)
    return img

def create_icon(image_path, icon_path, sizes):
    icon = Image.new('RGBA', (256, 256), (255, 255, 255, 0))  # Creating a blank RGBA image for the icon

    for size in sizes:
        resized_img = resize_image(image_path, size)
        offset = ((256 - size[0]) // 2, (256 - size[1]) // 2)  # Calculate the offset to center the image
        icon.paste(resized_img, offset)

    icon.save(icon_path)

if __name__ == "__main__":
    image_path = "otherworld.png"
    icon_path = "otherworld.ico"
    sizes = [(128, 128), (64, 64), (48, 48), (32, 32), (24, 24), (16, 16)]
    create_icon(image_path, icon_path, sizes)
