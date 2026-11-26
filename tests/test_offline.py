"""The app must work without internet: every script and stylesheet comes from static/vendor.
Only the web fonts may come from the internet (the page falls back to installed fonts)."""
import json
import os
import re
import threading
import unittest
import urllib.request

from helpers import ROOT

import server

STATIC = os.path.join(ROOT, "static")
ALLOWED_REMOTE = ("https://fonts.googleapis.com", "https://fonts.gstatic.com")


def references(page):
    html = open(os.path.join(STATIC, page), encoding="utf-8").read()
    refs = re.findall(r'<script[^>]+src="([^"]+)"', html) + re.findall(r'<link[^>]+href="([^"]+)"', html)
    m = re.search(r'<script type="importmap">(.*?)</script>', html, re.S)
    if m:
        refs += list(json.loads(m.group(1))["imports"].values())
    return refs


class OfflineTest(unittest.TestCase):
    def test_no_scripts_or_styles_from_the_internet(self):
        for page in ("index.html", "manual.html"):
            for ref in references(page):
                if ref.startswith(("http://", "https://", "//")):
                    self.assertTrue(ref.startswith(ALLOWED_REMOTE), f"{page} loads {ref} from the internet")

    def test_every_local_file_exists(self):
        for page in ("index.html", "manual.html"):
            for ref in references(page):
                if not ref.startswith(("http://", "https://", "//")):
                    path = os.path.normpath(os.path.join(STATIC, ref.lstrip("./")))
                    self.assertTrue(os.path.isfile(path), f"{page}: missing {ref}")

    def test_every_bundled_library_has_a_license_notice(self):
        vendor = os.path.join(STATIC, "vendor")
        notices = open(os.path.join(vendor, "THIRD_PARTY_NOTICES.md"), encoding="utf-8").read()
        for lib in os.listdir(vendor):
            if os.path.isdir(os.path.join(vendor, lib)):
                self.assertIn(f"`{lib}/", notices, f"static/vendor/{lib} is missing from THIRD_PARTY_NOTICES.md")

    def test_pages_have_a_tab_icon(self):
        for page in ("index.html", "manual.html"):
            html = open(os.path.join(STATIC, page), encoding="utf-8").read()
            self.assertRegex(html, r'<link rel="icon" href="favicon\.ico"', f"{page} has no tab icon")

    def test_app_imports_only_mapped_modules(self):
        app = open(os.path.join(STATIC, "app.js"), encoding="utf-8").read()
        imports = re.findall(r'^import .* from "([^"]+)";', app, re.M)
        mapped = json.loads(re.search(r'<script type="importmap">(.*?)</script>',
                                      open(os.path.join(STATIC, "index.html"), encoding="utf-8").read(),
                                      re.S).group(1))["imports"]
        for spec in imports:
            self.assertIn(spec, mapped, f"app.js imports {spec} which the import map does not provide")

    def test_vendor_files_are_served_with_the_right_type(self):
        srv = server.Server(("127.0.0.1", 0), server.Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            base = f"http://127.0.0.1:{srv.server_address[1]}"
            for path, ctype in (("/vendor/three/three.module.js", "javascript"),
                                ("/vendor/three/OrbitControls.js", "javascript"),
                                ("/vendor/codemirror/codemirror.min.js", "javascript"),
                                ("/vendor/codemirror/codemirror.min.css", "text/css"),
                                ("/favicon.ico", "image/"),
                                ("/favicon.png", "image/png")):
                with urllib.request.urlopen(base + path, timeout=5) as r:
                    self.assertEqual(r.status, 200, path)
                    self.assertIn(ctype, r.headers["Content-Type"], path)
        finally:
            srv.shutdown()


if __name__ == "__main__":
    unittest.main()
