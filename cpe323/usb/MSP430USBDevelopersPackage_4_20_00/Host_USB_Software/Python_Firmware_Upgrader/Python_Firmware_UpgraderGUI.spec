# -*- mode: python -*-
a = Analysis(['TargetGUI.py'],
             pathex=['./python-msp430-tools', 'C:\\msp430usb\\msp430_usb\\imports\\MSP430USBDevelopersPackage_4_20_00\\Host_USB_Software\\Python_Firmware_Upgrader'],
             hiddenimports=[],
             hookspath=None)
pyz = PYZ(a.pure)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          name=os.path.join('dist', 'Python_Firmware_UpgraderGUI.exe'),
          debug=False,
          strip=None,
          upx=True,
          console=False , icon='TI_Bug_Icon_Red.ico')
app = BUNDLE(exe,
             name=os.path.join('dist', 'Python_Firmware_UpgraderGUI.exe.app'))
