import sys,os
from imp import reload
import SDShortcutsEnhance.SDShortcutsEnhance as SDShortcutsEnhance

def initializeSDPlugin():
    reload(SDShortcutsEnhance)
    SDShortcutsEnhance.createMenu()
def uninitializeSDPlugin():
    pass