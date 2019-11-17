echo off
set DIR=Output\
set ROOT=%DIR%
:: for /R %ROOT% %%i in ( *.* ) do ( 
	:: echo %%i "%i%"
	:: del %%i
:: )
del /s/q Outlist\*.*
del /s/q Output\*.*
:: dir %ROOT%\*.hex /b
:: pause