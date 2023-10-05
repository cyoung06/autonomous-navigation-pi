import platform
from time import sleep

class WifiRouter:
    def __init__(self, bssid, ssid, rssi, channel):
        self.bssid:str = bssid
        self.ssid:str = ssid
        self.rssi:float = rssi
        self.channel:int = channel
    def __repr__(self):
        return f'WifiRouter {self.ssid} ({self.bssid}) strength {self.rssi} channel {self.channel}'


if platform.system() == 'Darwin':
    import objc

    objc.loadBundle('CoreWLAN',
                    bundle_path='/System/Library/Frameworks/CoreWLAN.framework',
                    module_globals=globals())
    objc.loadBundle('CoreLocation',
                    bundle_path='/System/Library/Frameworks/CoreLocation.framework',
                    module_globals=globals())
    mgr = CLLocationManager.alloc().init()

    print(CLLocationManager.locationServicesEnabled())
    print(CLLocationManager.authorizationStatus())
    mgr.delegate()
    mgr.requestAlwaysAuthorization()
    mgr.startUpdatingLocation()
    max_wait = 60
    # Get the current authorization status for Python
    for i in range(1, max_wait):
        authorization_status = mgr.authorizationStatus()
        print(mgr.authorizationStatus())
        if authorization_status == 3 or authorization_status == 4:
            print("Python has been authorized for location services")
            break
        if i == max_wait - 1:
            exit("Unable to obtain authorization, exiting")
        sleep(1)
else:
    import iw_parse

def get_nearby_routers():
    def get_nearby_routers_macos():
        for iname in CWInterface.interfaceNames():
            interface = CWInterface.interfaceWithName_(iname)
            networks, error = interface.scanForNetworksWithName_error_(None, None)

            list = []
            for network in networks:
                list.append(WifiRouter(
                    network.bssid(),
                    network.ssid(),
                    network.rssi(),
                    network.channel()
                ))
            return list
    def get_nearby_routers_rpi():
        networks = iw_parse.get_interfaces(interface='wlan0')

        list = []
        for network in networks:
            list.append(WifiRouter(
                network['Address'],
                network['Name'],
                -int(network['Signal Level']),
                int(network['Channel'])
            ))
        return list

    if platform.system() == 'Darwin':
        return get_nearby_routers_macos()
    else:
        return get_nearby_routers_rpi()
    pass
