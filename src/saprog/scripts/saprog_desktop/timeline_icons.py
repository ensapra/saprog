"""Small native Tk icons for the timeline toolbar."""


_PIXEL_ART = {
    "new": (
        "................",
        "....##########..",
        "....#++++++++#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#+######+#..",
        "....#++++++++#..",
        "....##########..",
        "................",
    ),
    "load": (
        "................",
        "..YYYY..........",
        "..YyyyYYYY......",
        "..YyyyyyyyY.....",
        "..############..",
        "..#fffffffffff#..",
        "..#fffffffYfff#..",
        "..#ffffffYYfff#..",
        "..#fffffYYYYff#..",
        "..#ffffYYYYYff#..",
        "..#fffYYYYYYYf#..",
        "..#ffYYYYYYYYY#..",
        "..#fYYYYYYYYYY#..",
        "..############..",
        "................",
        "................",
    ),
    "save": (
        "..BBBBBBBBBBBB..",
        "..BbbbbbbbbbbB..",
        "..BbbWWWWWWbbB..",
        "..BbbW....WbbB..",
        "..BbbW....WbbB..",
        "..BbbWWWWWWbbB..",
        "..BbbbbbbbbbbB..",
        "..BBBBBBBBBBBB..",
        "..BB..........B..",
        "..BB..........B..",
        "..BB..WWWWWW..B..",
        "..BB..W....W..B..",
        "..BB..W....W..B..",
        "..BB..WWWWWW..B..",
        "..BBBBBBBBBBBB..",
        "................",
    ),
    "play": (
        "................",
        "...BB...........",
        "...BBBB.........",
        "...BBBBBB.......",
        "...BBBBBBBB.....",
        "...BBBBBBBBBB...",
        "...BBBBBBBBBBBB.",
        "...BBBBBBBBBBBB.",
        "...BBBBBBBBBB...",
        "...BBBBBBBB.....",
        "...BBBBBB.......",
        "...BBBB.........",
        "...BB...........",
        "................",
        "................",
        "................",
    ),
    "test": (
        "................",
        "...GG...........",
        "...G..G.........",
        "...G....G.......",
        "...G......G.....",
        "...G........G...",
        "...G..........G.",
        "...G..........G.",
        "...G........G...",
        "...G......G.....",
        "...G....G.......",
        "...G..G.........",
        "...GG...........",
        "................",
        "................",
        "................",
    ),
    "stop": (
        "................",
        "...RRRRRRRRRR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RrrrrrrrrR...",
        "...RRRRRRRRRR...",
        "................",
        "................",
        "................",
    ),
}


def create_timeline_icons(root, tk_module):
    """Return retained PhotoImages for the timeline's font-independent icons."""
    colors = {
        "#": "#e5edf7",
        "+": "#498bd1",
        "Y": "#f2c14e",
        "y": "#d99a2b",
        "f": "#26364b",
        "B": "#70b5f5",
        "b": "#245c96",
        "W": "#e5edf7",
        "G": "#86e3ad",
        "R": "#ff9c96",
        "r": "#a94442",
    }
    images = {}
    for name, rows in _PIXEL_ART.items():
        image = tk_module.PhotoImage(master=root, width=32, height=32)
        for y, row in enumerate(rows):
            for x, pixel in enumerate(row):
                if x >= 16:
                    break
                color = colors.get(pixel)
                if color:
                    image.put(color, to=(x * 2, y * 2, x * 2 + 2, y * 2 + 2))
        images[name] = image
    return images
