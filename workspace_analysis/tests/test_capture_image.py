"""Pixel QA rejects unavailable display captures before accepting evidence."""
import importlib.util
from pathlib import Path
import pytest


@pytest.mark.parametrize('kind',['null','black','varied'])
def test_capture_pixel_content_guard(kind):
    from PyQt5.QtGui import QImage,QColor
    path=Path(__file__).resolve().parents[1]/'repro/scripts/capture_replay.py'
    spec=importlib.util.spec_from_file_location('capture_replay_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    image=QImage() if kind=='null' else QImage(60,60,QImage.Format_RGB32)
    if kind!='null':image.fill(QColor('black'))
    if kind=='varied':
        for x in range(60):
            for y in range(60):
                image.setPixelColor(x,y,QColor('red') if x<20 else QColor('green') if x<40 else QColor('blue'))
    assert module.image_has_content(image)==(kind=='varied')
