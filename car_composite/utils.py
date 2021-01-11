'''
   Copyright 2021 Kerry Johnson

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
'''
import subprocess

def ip(remove = '10.'):
    all_ips = subprocess.check_output(['hostname', '-I'])
    all_ips = [ip.decode('utf-8') for ip in all_ips.split()]
    
    if remove is not None:
        # Remove (e.g.) Docker/Resin IPs
        all_ips = [ip for ip in all_ips if not ip.startswith(remove)]
    
    return all_ips[0] if len(all_ips) > 0 else None