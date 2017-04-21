import device
import handlers.drawer

## Maps dye names to colors to use for those dyes.
DYE_TO_COLOR = {
        'Cy5': (0, 255, 255),
        'DAPI': (184, 0, 184),
        'DIC': (128, 128, 128),
        'FITC': (80,255,150),
        'GFP': (0, 255, 0),
        'mCherry': (255, 0, 0),
        'RFP': (255, 0, 0),
        'Rhod': (255,80,20),
        'YFP': (255, 255, 0),
}

CLASS_NAME = 'DrawerDevice'

class DrawerDevice(device.Device):
    def getHandlers(self):
        # Note that these names have to be the same as the names used for the
        # CameraHandler instances created by other devices.
        cameraNames = ('West', 'Northwest', 'Northeast', 'East')
        drawerNames = ('C drawer', 'B drawer')
        dyes = [('GFP', 'Cy5', 'mCherry', 'DAPI'),
                ('Cy5', 'FITC', 'Rhod', 'DAPI')]
        wavelengths = [(525, 670, 585, 447),
                (695, 518, 590, 450)]
        settings = []
        for i, name in enumerate(drawerNames):
            colors = [DYE_TO_COLOR[d] for d in dyes[i]]
            settings.append(handlers.drawer.DrawerSettings(
                name, cameraNames, dyes[i], colors, wavelengths[i])
            )
        return [handlers.drawer.DrawerHandler("drawer", "miscellaneous",
                settings, 0)]
