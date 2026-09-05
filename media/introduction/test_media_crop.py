"""A video comparison must preserve exact coordinates across compressed tile edges."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
import tifffile

from build_film import tiff_crop, still_frame


class TiledCropTest(unittest.TestCase):
    def test_crop_matches_pixels_across_tiles_and_partial_last_tile(self):
        values=np.random.default_rng(73).integers(0,256,(79,103),dtype=np.uint8)
        with tempfile.TemporaryDirectory(prefix='climbot_media_crop_') as directory:
            path=Path(directory)/'source.tif'
            tifffile.imwrite(path,values,tile=(32,32),compression='deflate')
            for box in [(0,0,103,79),(13,17,91,65),(96,70,103,79)]:
                x0,y0,x1,y1=box
                np.testing.assert_array_equal(np.asarray(tiff_crop(path,box)),values[y0:y1,x0:x1])
            with self.assertRaises(ValueError):
                tiff_crop(path,(-1,0,32,32))

    def test_push_in_has_subpixel_motion_without_integer_crop_jumps(self):
        y,x=np.mgrid[:240,:320]
        image=np.exp(-((x-230.)**2+(y-80.)**2)/18).astype(np.float32)
        centres=[]
        for index in range(90):
            pixels=still_frame(image,index,90)
            centre=float((pixels*x).sum()/pixels.sum())
            expected=159.5+(230-159.5)*(1.015+.045*index/89)
            self.assertLess(abs(centre-expected),.06)
            centres.append(centre)
        self.assertLess(float(np.max(np.abs(np.diff(centres)))),.10)


if __name__=='__main__':unittest.main()
