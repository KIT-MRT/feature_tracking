# matches_msg_ros
* Message definitions to communicate matches between nodes.
* we have two matches messages that can be transmitted, one without outlier flags one with outlier flags
* tracklets are assorted from current to old, so matches_msg.data[0].data[0] is the most current keypoint of the first tracklet
* timestamps have the same length as the longest tracklet and are assorted from most recent to oldest therefore ```match[tracklet_size-i] of some tracklet -> timestamps[timestamps.size()-i]``` holds
* matches shall be stored as m.data[0]=u, m.data[1]=v
## Usage

* Use the structure
* you can show them with the matches_msg_viewer_ros_tool

## History
* added scaling for matching
* added timestamps per msg that correspond to matches. match[tracklet_size-i] of some tracklet corresponds to timestamps[timestamps.size()-i]
* exposed tracklet ids from feature_tracking

## Credits

* Johannes Gräter

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