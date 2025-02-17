vgc_add_library(ui
    THIRD_DEPENDENCIES
        ${VGC_QT_GUI}
        ${VGC_QT_OPENGL}

    VGC_DEPENDENCIES
        graphics
        dom
        workspace

    MACOS_DEPENDENCIES
        "-framework ApplicationServices"

    CPP_HEADER_FILES
        action.h
        actiongroup.h
        api.h
        application.h
        boolsetting.h
        boolsettingedit.h
        button.h
        column.h
        command.h
        cursor.h
        dialog.h
        event.h
        exceptions.h
        focus.h
        flex.h
        genericaction.h
        genericcommands.h
        grid.h
        iconwidget.h
        imagebox.h
        key.h
        keyevent.h
        label.h
        lineedit.h
        logcategories.h
        macospermissions.h
        margins.h
        menu.h
        menubutton.h
        messagedialog.h
        modifierkey.h
        mousebutton.h
        mouseevent.h
        numberedit.h
        numbersetting.h
        numbersettingedit.h
        overlayarea.h
        panel.h
        panelarea.h
        plot2d.h
        popuplayer.h
        preferredsizecalculator.h
        qtutil.h
        row.h
        scrollevent.h
        settingedit.h
        setting.h
        settings.h
        shortcut.h
        strings.h
        tabbar.h
        tabbody.h
        toggle.h
        tooltip.h
        widget.h
        window.h

        detail/layoututil.h
        detail/paintutil.h
        detail/qopenglengine.h

    CPP_SOURCE_FILES
        action.cpp
        actiongroup.cpp
        application.cpp
        boolsetting.cpp
        boolsettingedit.cpp
        button.cpp
        column.cpp
        command.cpp
        cursor.cpp
        dialog.cpp
        event.cpp
        exceptions.cpp
        focus.cpp
        flex.cpp
        genericaction.cpp
        genericcommands.cpp
        grid.cpp
        iconwidget.cpp
        imagebox.cpp
        key.cpp
        keyevent.cpp
        label.cpp
        lineedit.cpp
        logcategories.cpp
        margins.cpp
        menu.cpp
        menubutton.cpp
        messagedialog.cpp
        modifierkey.cpp
        mousebutton.cpp
        mouseevent.cpp
        numberedit.cpp
        numbersetting.cpp
        numbersettingedit.cpp
        overlayarea.cpp
        panel.cpp
        panelarea.cpp
        plot2d.cpp
        popuplayer.cpp
        preferredsizecalculator.cpp
        qtutil.cpp
        row.cpp
        scrollevent.cpp
        settingedit.cpp
        setting.cpp
        settings.cpp
        shortcut.cpp
        strings.cpp
        tabbar.cpp
        tabbody.cpp
        toggle.cpp
        tooltip.cpp
        widget.cpp
        window.cpp

        detail/paintutil.cpp
        detail/qopenglengine.cpp

    OBJCPP_SOURCE_FILES
        macospermissions.mm

    RESOURCE_FILES
        stylesheets/default.vgcss
)

add_subdirectory(wraps)
add_subdirectory(tests)
