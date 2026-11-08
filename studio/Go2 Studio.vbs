' Go2 Studio. Double-click to open.
'
' Starts studio.py with Windows' own Python and no console window; it opens
' the Studio window by itself. If Studio is already running, this just opens
' another window onto it instead of starting a second copy.
'
' A .vbs rather than a .bat because a .bat always flashes a black console
' window, which is exactly the terminal this is meant to spare people.

Option Explicit
Dim fso, sh, here
Set fso = CreateObject("Scripting.FileSystemObject")
Set sh = CreateObject("WScript.Shell")
here = fso.GetParentFolderName(WScript.ScriptFullName)

On Error Resume Next
sh.Run "pythonw """ & here & "\studio.py""", 0, False
If Err.Number <> 0 Then
  MsgBox "Go2 Studio could not start." & vbCrLf & vbCrLf & _
         "It needs Python for Windows (python.org, 3.10 or newer), with" & vbCrLf & _
         "'Add python.exe to PATH' ticked when installing." & vbCrLf & vbCrLf & _
         Err.Description, vbExclamation, "Go2 Studio"
End If
