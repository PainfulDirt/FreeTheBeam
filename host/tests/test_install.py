"""host/install.sh into a temporary root (ROOT=, SYSTEMCTL=true,
UDEVADM=true), as the current user: what it installs, and that an existing
k1-lidar helper (a K1's, on the same host) is left alone."""
import getpass
import os
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "install.sh")


class TestInstall(unittest.TestCase):
    def fake_usb(self, root, serials):
        """CH343 devices (1a86:55d3) in a fake /sys/bus/usb/devices."""
        for i, sn in enumerate(serials):
            d = os.path.join(root, "sys/bus/usb/devices/1-%d" % (i + 1,))
            os.makedirs(d)
            for k, v in (("idVendor", "1a86"), ("idProduct", "55d3"),
                         ("serial", sn)):
                with open(os.path.join(d, k), "w") as f:
                    f.write(v + "\n")

    def run_install(self, root, *args, usb=("5A1F012345",)):
        if usb is not None:
            self.fake_usb(root, usb)
        env = dict(os.environ, ROOT=root, SYSTEMCTL="true", UDEVADM="true")
        return subprocess.run(["sh", SCRIPT, getpass.getuser()] + list(args),
                              env=env, capture_output=True, text=True)

    def read(self, root, path):
        with open(os.path.join(root, path.lstrip("/"))) as f:
            return f.read()

    def test_helper_install_keeps_existing_k1_helper(self):
        with tempfile.TemporaryDirectory() as root:
            unit_dir = os.path.join(root, "etc/systemd/system")
            os.makedirs(unit_dir)
            with open(os.path.join(unit_dir, "k1-lidar.service"), "w") as f:
                f.write("K1 HELPER\n")
            r = self.run_install(root, "--helper")
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(self.read(root, "/etc/systemd/system/"
                                       "k1-lidar.service"), "K1 HELPER\n")
            drv = self.read(root, "/etc/systemd/system/lidar-direct.service")
            self.assertIn("User=%s\n" % getpass.getuser(), drv)
            hlp = self.read(root, "/etc/systemd/system/lidar-helper.service")
            self.assertIn("--socket /tmp/ai_server_uds", hlp)
            self.assertIn("--listen 127.0.0.1:7132", hlp)
            self.assertIn("--data /var/lib/lidar-helper", hlp)
            self.assertIn("User=%s\n" % getpass.getuser(), hlp)
            self.assertTrue(os.path.exists(os.path.join(
                root, "opt/freethebeam/driver/lidar-direct.py")))
            self.assertFalse(os.path.exists(os.path.join(
                root, "opt/freethebeam/driver/tests")))
            self.assertTrue(os.path.exists(os.path.join(
                root, "opt/freethebeam/helper/lidar_service.py")))
            self.assertTrue(os.path.exists(os.path.join(
                root, "etc/udev/rules.d/60-creality-lidar.rules")))


    def test_udev_rule_pinned_to_the_module(self):
        with tempfile.TemporaryDirectory() as root:
            r = self.run_install(root)
            self.assertEqual(r.returncode, 0, r.stderr)
            rule = self.read(root, "/etc/udev/rules.d/60-creality-lidar.rules")
            self.assertIn('ATTRS{serial}=="5A1F012345"', rule)

    def test_no_or_several_modules_refused(self):
        for usb in ((), ("AAA", "BBB")):
            with tempfile.TemporaryDirectory() as root:
                r = self.run_install(root, usb=usb)
                self.assertNotEqual(r.returncode, 0)
                self.assertIn("SERIAL=", r.stderr)
                self.assertFalse(os.path.exists(os.path.join(
                    root, "etc/systemd/system/lidar-direct.service")))

    def test_serial_given(self):
        with tempfile.TemporaryDirectory() as root:
            env = dict(os.environ, ROOT=root, SYSTEMCTL="true",
                       UDEVADM="true", SERIAL="BBB")
            self.fake_usb(root, ("AAA", "BBB"))
            r = subprocess.run(["sh", SCRIPT, getpass.getuser()], env=env,
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('ATTRS{serial}=="BBB"', self.read(
                root, "/etc/udev/rules.d/60-creality-lidar.rules"))


if __name__ == "__main__":
    unittest.main()
