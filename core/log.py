import pymel.core as pm

def log_window():
    logWin = pm.window(title="tRigger Log")
    pm.columnLayout(adjustableColumn=True)
    pm.cmdScrollFieldReporter(width=800, height=500, clr=True)
    pm.button(label='Close', command=(
        'import pymel.core as pm\npm.deleteUI(\"' + logWin +
        '\", window=True)'))
    pm.setParent('..')
    pm.showWindow(logWin)
    print 'Hello from tRigger!'