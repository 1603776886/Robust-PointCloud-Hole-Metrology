#ifndef MyAppVersion
  #define MyAppVersion "0.1.0-dev"
#endif
#ifndef SourceDir
  #error SourceDir is required
#endif
#ifndef OutputDir
  #error OutputDir is required
#endif
#ifndef VCRedist
  #error VCRedist is required
#endif
#ifndef RepoRoot
  #error RepoRoot is required
#endif

#define MyAppName "Robust Hole Metrology"
#define MyAppExeName "RobustHoleMetrology.exe"

[Setup]
AppId={{9323558F-C92A-4C45-A57B-BF6FF3E0A707}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher=Robust Hole Metrology Project
DefaultDirName={autopf}\Robust Hole Metrology
DefaultGroupName=Robust Hole Metrology
DisableProgramGroupPage=yes
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir={#OutputDir}
OutputBaseFilename=RobustHoleMetrology-Dev-{#MyAppVersion}-Setup-x64
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\{#MyAppExeName}
SetupLogging=yes
LicenseFile={#RepoRoot}\LICENSE

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Additional shortcuts:"; Flags: unchecked

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#VCRedist}"; DestDir: "{tmp}"; DestName: "vc_redist.x64.exe"; Flags: deleteafterinstall

[Icons]
Name: "{autoprograms}\Robust Hole Metrology"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"
Name: "{autodesktop}\Robust Hole Metrology"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{tmp}\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ Runtime..."; Flags: waituntilterminated
Filename: "{app}\{#MyAppExeName}"; Description: "Launch Robust Hole Metrology"; WorkingDir: "{app}"; Flags: nowait postinstall skipifsilent
