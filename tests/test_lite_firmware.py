"""Compile and execute the actual sketch with desktop Arduino I/O doubles."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


def desktop_compiler():
    environment = os.environ.copy()
    for name in (os.environ.get('CXX', ''), 'g++', 'clang++'):
        if name and (executable := shutil.which(name)):
            return executable, environment, False
    if os.name == 'nt':
        for variable in ('ProgramFiles(x86)', 'ProgramFiles'):
            root = Path(os.environ.get(variable, 'C:/Program Files')) / 'Microsoft Visual Studio'
            for script in sorted(root.glob('*/*/VC/Auxiliary/Build/vcvars64.bat'), reverse=True):
                # vcvars is the installed MSVC environment setup, not repo input.
                result = subprocess.run(f'call "{script}" >nul && set', shell=True,
                                        capture_output=True, text=True, timeout=30)
                if result.returncode:
                    continue
                environment = dict(line.split('=', 1) for line in result.stdout.splitlines()
                                   if '=' in line and not line.startswith('='))
                executable = shutil.which('cl.exe', path=environment.get('Path', environment.get('PATH', '')))
                if executable:
                    return executable, environment, True
    return None


class FirmwareTests(unittest.TestCase):
    def test_heartbeat_behavior_in_real_firmware(self):
        compiler = desktop_compiler()
        if compiler is None:
            self.skipTest('Firmware tests require a desktop C++ compiler (GCC, Clang, or MSVC).')
        executable, environment, msvc = compiler
        sources = Path(__file__).resolve().parents[1] / 'firmware' / 'tests'
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / ('heartbeat.exe' if os.name == 'nt' else 'heartbeat')
            if msvc:
                command = [executable, '/nologo', '/std:c++14', '/EHsc', '/W4',
                           f'/I{sources}', str(sources / 'test_heartbeat.cpp'),
                           f'/Fe:{binary}', f'/Fo:{Path(folder) / "heartbeat.obj"}']
            else:
                command = [executable, '-std=c++14', '-Wall', '-Wextra', '-I', str(sources),
                           str(sources / 'test_heartbeat.cpp'), '-o', str(binary)]
            build = subprocess.run(command, env=environment, cwd=folder, capture_output=True, text=True, timeout=60)
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn('8 firmware heartbeat scenarios passed', result.stdout)
