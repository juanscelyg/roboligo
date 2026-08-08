# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

class TestRoboligoCli(unittest.TestCase):
    """Basic tests for roboligo_cli package."""

    def test_import(self):
        """Test that the package can be imported."""
        try:
            import roboligo_cli
        except ImportError:
            self.fail("Failed to import roboligo_cli")


if __name__ == '__main__':
    unittest.main()