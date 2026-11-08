"""Draw Studio's icon: "G2" in green on the near-black of the app.

    python make_icon.py        writes studio.ico beside this file

Needs Pillow and Windows' Bahnschrift font (the same one the page uses), so
it is a development step, not something Studio runs. The .ico is kept in the
repo.
"""
import os

from PIL import Image, ImageDraw, ImageFont

GROUND = (11, 17, 13, 255)
GREEN = (79, 224, 138, 255)
FONT = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", "bahnschrift.ttf")


def draw(size):
    k = 4                                            # draw big, shrink: smooth edges
    s = size * k
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((0, 0, s - 1, s - 1), radius=int(s * .2), fill=GROUND)
    font = ImageFont.truetype(FONT, int(s * .56))
    try:
        font.set_variation_by_name("Bold")
    except (OSError, ValueError, AttributeError):
        pass
    d.text((s / 2, s / 2 + s * .02), "G2", font=font, fill=GREEN, anchor="mm")
    return img.resize((size, size), Image.LANCZOS)


def main():
    sizes = [16, 24, 32, 48, 64, 128, 256]
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "studio.ico")
    big = draw(256)
    big.save(out, sizes=[(n, n) for n in sizes], append_images=[draw(n) for n in sizes[:-1]])
    big.save(os.path.join(os.path.dirname(out), "icon-preview.png"))
    print("wrote", out)


if __name__ == "__main__":
    main()
