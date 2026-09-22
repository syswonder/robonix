# SPDX-License-Identifier: MulanPSL-2.0
"""The shape of the pictures kept for an object.

The rect handed to the store is a 3D bounding box projected into the image,
so a tabletop seen from across it, or a picture frame edge-on, projects to a
band a few pixels tall. Stored raw, that is what the panel showed: a sliver
of wood on black. These pictures exist to confirm that the object on the
screen is the one you mean, and a sliver confirms nothing.
"""
from scene_service.object_views import (_MAX_CROP_ASPECT, crop_quality,
                                        pad_rect)


def _aspect(rect):
    w, h = rect[2] - rect[0], rect[3] - rect[1]
    return max(w, h) / float(max(1, min(w, h)))


def test_a_projected_band_becomes_something_recognisable():
    """The complaint, in one assertion: a tabletop's projection is 400x12
    and what gets stored has to be a picture of a table, not a line."""
    padded = pad_rect((100, 300, 500, 312), 640, 480)
    assert _aspect(padded) <= _MAX_CROP_ASPECT + 0.01
    # And still centred on the object it was cut for.
    assert padded[0] <= 100 and padded[2] >= 500


def test_a_square_object_is_only_given_context():
    """Nothing to correct, so the only change is the margin -- which is
    itself worth having: a crop cut exactly at the outline is harder to
    place than one showing what the object sits on."""
    padded = pad_rect((200, 200, 300, 300), 640, 480)
    assert padded[0] < 200 and padded[1] < 200
    assert padded[2] > 300 and padded[3] > 300
    assert _aspect(padded) <= 1.05


def test_the_image_edge_wins_over_the_ratio():
    """An off-centre object is still recognisable; one framed somewhere
    else is not. Clamping never slides the crop off its subject."""
    padded = pad_rect((0, 0, 400, 12), 640, 480)
    assert padded[0] == 0 and padded[1] == 0
    assert padded[2] <= 640 and padded[3] <= 480
    # It grew downward, since that is the only direction available.
    assert padded[3] > 12


def test_a_strip_that_cannot_be_padded_scores_below_a_whole_object():
    """Otherwise a wide band against an image edge takes a slot from a crop
    showing the entire thing, on area alone."""
    strip = (0, 200, 600, 240)      # 600x40 against the left edge
    whole = (100, 100, 300, 300)    # 200x200, clear of every edge
    assert crop_quality(strip, 640, 480) < crop_quality(whole, 640, 480)


def test_padding_never_leaves_the_image():
    for rect in [(0, 0, 40, 40), (600, 440, 640, 480), (300, 0, 340, 8)]:
        u0, v0, u1, v1 = pad_rect(rect, 640, 480)
        assert 0 <= u0 < u1 <= 640
        assert 0 <= v0 < v1 <= 480
