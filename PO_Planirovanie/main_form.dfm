object mainform: Tmainform
  Left = 178
  Top = 123
  BorderStyle = bsSingle
  Caption = #1057#1080#1089#1090#1077#1084#1072' '#1055#1083#1072#1085#1080#1088#1086#1074#1072#1085#1080#1103' '#1055#1086#1083#1077#1090#1072' "'#1057#1055#1055' - '#1070#1076#1072#1083'"'
  ClientHeight = 139
  ClientWidth = 452
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  Icon.Data = {
    000001000200101008000000000028010000260000002020080000000000E802
    00004E0100002800000010000000200000000100040000000000C00000000000
    0000000000001000000000000000000000000000800000800000008080008000
    00008000800080800000C0C0C000808080000000FF0000FF000000FFFF00FF00
    0000FF00FF00FFFF0000FFFFFF00000000000000000000000000000000008777
    7777777777708FFFFFFFFFFFFF708FF0FFFFFFFFFF708FF00FFF00000F708000
    00FF97979F708FF00FFFFFFFFF708FF0FF0000000F708FFFFF6767676F708FFF
    FFFFFFFFFF708FF0000000000F708FF7A7A7A7A7AF708FFFFFFFFFFFFF708888
    8888888888800000000000000000FFFFFFFF0000FFFF0000FFFF0000FFFF0000
    FFFF0000FFFF0000FFFF0000FFFF0000FFFF0000FFFF0000FFFF0000FFFF0000
    FFFF0000FFFF0000FFFFFFFFFFFF280000002000000040000000010004000000
    0000800200000000000000000000100000000000000000000000000080000080
    000000808000800000008000800080800000C0C0C000808080000000FF0000FF
    000000FFFF00FF000000FF00FF00FFFF0000FFFFFF0000000000000000000000
    0000000000000000000000000000000000000000000000000000000000000000
    0000000000000000000000000000000000000000000008888888888888888888
    888888888880087FFFFFFFFFFFFFFFFFFFFFFFFFFF80087FFFFFFFFFFFFFFFFF
    000000000F80087FFFFFFFFFFFFFFFFF797979790F80087FFFFF00FFFFFFFFFF
    979797970F80087FFFFF000FFFFFFFFFFFFFFFFFFF80087F00000000FFFF0000
    000000000F80087F000000000FFF7676767676760F80087F00000000FFFF6767
    676767670F80087FFFFF000FFFFFFFFFFFFFFFFFFF80087FFFFF00FF00000000
    000000000F80087FFFFFFFFF73737373737373730F80087FFFFFFFFF37373737
    373737370F80087FFFFFFFFFFFFFFFFFFFFFFFFFFF80087FFFFF000000000000
    000000000F80087FFFFFD7D7D7D7D7D7D7D7D7D70F80087FFFFF7D7D7D7D7D7D
    7D7D7D7D0F80087FFFFFFFFFFFFFFFFFFFFFFFFFFF80087F0000000000000000
    000000000F80087FA7A7A7A7A7A7A7A7A7A7A7A70F80087F7A7A7A7A7A7A7A7A
    7A7A7A7A0F80087FFFFFFFFFFFFFFFFFFFFFFFFFFF8008777777777777777777
    7777777777800888888888888888888888888888888000000000000000000000
    0000000000000000000000000000000000000000000000000000000000000000
    00000000000000000000000000000000000000000000FFFFFFFFFFFFFFFFFFFF
    FFFF800000008000000080000000800000008000000080000000800000008000
    0000800000008000000080000000800000008000000080000000800000008000
    0000800000008000000080000000800000008000000080000000800000008000
    000080000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF}
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object GroupBox1: TGroupBox
    Left = 8
    Top = 8
    Width = 433
    Height = 121
    Caption = ' '#1042#1093#1086#1076' '#1074' '#1089#1080#1089#1090#1077#1084#1091' '
    TabOrder = 0
    object Label1: TLabel
      Left = 96
      Top = 24
      Width = 96
      Height = 13
      Caption = #1048#1084#1103' '#1087#1086#1083#1100#1079#1086#1074#1072#1090#1077#1083#1103
    end
    object Label2: TLabel
      Left = 96
      Top = 56
      Width = 38
      Height = 13
      Caption = #1055#1072#1088#1086#1083#1100
    end
    object Button1: TButton
      Left = 96
      Top = 88
      Width = 75
      Height = 25
      Caption = #1042#1061#1054#1044
      TabOrder = 0
      OnClick = Button1Click
    end
    object login: TEdit
      Left = 201
      Top = 21
      Width = 121
      Height = 21
      TabOrder = 1
    end
    object password: TEdit
      Left = 201
      Top = 53
      Width = 121
      Height = 21
      PasswordChar = '*'
      TabOrder = 2
    end
  end
end
