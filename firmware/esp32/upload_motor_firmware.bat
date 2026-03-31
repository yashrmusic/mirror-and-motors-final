@echo off
echo ========================================
echo Motor Firmware Upload to ESP32
echo ========================================
echo.
echo IMPORTANT: Put ESP32 in bootloader mode:
echo   1. Hold the BOOT button (keep holding)
echo   2. Press and release RESET button
echo   3. Release BOOT button
echo.
echo Press any key when ESP32 is in bootloader mode...
pause
echo.
echo Detecting ESP32 type and uploading...
echo.

cd /d "%~dp0"

REM Try ESP32 first (most common)
echo Trying ESP32...
pio run -e esp32 --target upload --upload-port COM8
if %ERRORLEVEL% EQU 0 (
    goto success
)

REM If ESP32 fails, try ESP32-S3
echo Trying ESP32-S3...
pio run -e esp32s3 --target upload --upload-port COM8
if %ERRORLEVEL% EQU 0 (
    goto success
)

goto failed

:success
echo.
echo ========================================
echo Upload Successful!
echo ========================================
echo Motor firmware is now on ESP32
echo You can now run the motor GUI
goto end

:failed
echo.
echo ========================================
echo Upload Failed!
echo ========================================
echo Make sure:
echo   1. ESP32 is in bootloader mode
echo   2. COM8 is the correct port
echo   3. USB cable is connected
echo   4. No other program is using COM8
goto end

:end
pause

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Upload Successful!
    echo ========================================
    echo Motor firmware is now on ESP32
    echo You can now run the motor GUI
) else (
    echo.
    echo ========================================
    echo Upload Failed!
    echo ========================================
    echo Make sure:
    echo   1. ESP32 is in bootloader mode
    echo   2. COM8 is the correct port
    echo   3. USB cable is connected
)

pause

