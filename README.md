# Feature Tracking

This is a wrapper for feature tracking in consecutive frames. Right now, it allows LibViso mono and stereo matches to be tracked.

## Usage

* Usage is obvious: check out, take header, read comments. Use.
* pushBack images to Buffer, match them, retrieve tracklets
* you can specify a mask cv::Mat(rows,cols,CV_8UC1) for defining regions in which matching shall not be done. All maxima that have a mask value of 0 are be deleted. 

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits

TODO: Write credits

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
