#ifndef A16_HDR
#define A16_HDR
#include <vcl.h>

static bool arg_work_pr; // ������� ������ ������

static double ArgonMemoryType[4096];

static struct {
byte mode;
int addr;
int value;
char z;
} irvi_type;

static struct {
double b1b2;  // �1 + �2
double k1k2;  // �1 + �2
double halfk; // �������� �
double verch; // ������� ������� ��
double sred;  // ������� ������� ��
double niz;   // ������ ������� ��
} dopusk_zu;  // ������� �� �������� ��������� (������������)

static struct {
double Ex; //  ��������� � ���������� ������ x
double Ey; //  ��������� � ������� �� ������� y
double Ez; //  ��������� � ������� �� �������� z

double ax; // ��������� � ���������� ������ x
double ay; // ��������� � ������� �� ������� y
double az; // ��������� � ������� �� �������� z

double axd;
double axruo;

double rs;   // ��������� ����������
double sks;  // �������� ����������

double V;
double vb;
double vby, vbz;
double om;

double omyx; // ������� �������� �� �� ����������� (��� ��) Ex
double omzx; // ������� �������� �� ������� (��� OZ) Ez
double omx;  // ������� �������� �������� ������ ��� ��
double omy;  // ������� �������� �� �� ����������� (��� ��)
double omz;  // ������� �������� �� ������� (��� OZ)
double omyf; //
double omzf; //
double uomx; //
double uomy; //
double uomz; //

double ypr;  // ������ Sy
double zpr;  // ������ Sz
double Spr;  // ������ S

double vbok;   //

double rasp;   //
double rudkg;  // ������� �� ���
double ruokgx; // ROU x
double ruokgy; // ROU y
double ruokgz; // ROU z

double kvkg;   //

double ax1;
double Y2;
}dynamics;

static bool argon_auto_contr; // ������� ��������������� ���������� ���.
static bool dock_fbvs;        // ������� �������� �� ����
static bool can_send_toirvi;  // ���� ���������� ������ �� ����

static struct{
byte str_1;
byte str_2;
byte str_3;
byte str_4;
}bfi_strings;

static int i_tok;

static struct{
bool dpo_x;
}dpo_bit;

static bool dpo_status_bit;

static byte ht = 1;

static int i, j, K, k;  // ����� ����
static long i_ot_pusk;  // ����� ���� �� �����
static double dt;       // Delta t

static int GSO_types;
static double deltavt_1, deltavt_2;

//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\
// ���������� - �������������� � ����������!
static int a_debugger; // ���������� �� �������� ���������� � ������.
static int i_012;
//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\//\\\

static AnsiString irvi_string = "              ";
static AnsiString irvi_string_i[4];
static bool irvi_err;

static TDateTime gc1_time;

static double v_tek_m;

/////////////////////////////////////
// �������� ��� ������� ���������� //
//    ���������� ��� � ���         //
/////////////////////////////////////
static double r_m[32];
static double v_b1b2[32];
static double v_k1k2[32];
static double v_half_k1k2[32];
static double v_verch[32];
static double v_sred[32];
static double v_niz[32];

/////////////////////////////
// ��������� ��������� ��� //
// � ������ �������������  //
/////////////////////////////
static const double ax_b1b2 = 0.07396;  //� �������� 0.0778 ��� � ��, � ��� 0,093
static const double ax_k1k2 = 0.03698;
static const double ax_half = 0.01849;
static const double ax_verh = 0.00711;
static const double ax_sred = 0.00396;
static const double ax_nijn = 0.00254;

#endif //A16_HDR

/*

*********************
* � � � � � � � � � *
*  � � � � � - 1 6  *
*********************

���� - ��������� ���������� �������������� ��������� (��������� ���������):
- ������������� ��������� ������� ��� ������ ��;
- ������������� � ������� ������ �� ��������� ��������� (����, ����, ����, ����, ����);
- ���������� ������ �������� �� � ������������ � ����������� ��������, �������� ��������, ������������;
- ���������� ������ ������ ��� ��� ����������� ������ ����;
- ������ ����� ���������� �� � ���.

���� - ��������� ���������� ����������� ����:
- �������� � ��������� ���������� ����;
- ������ ������� � ���������� ���� �� ���������� ������ � ������������ ��� �� ����������.
  ������������ ������ ��������� ������� ���������� ��� ���� � �� ������ ����������� ���������� ���� ��������� ����,
  ���� �������� ��� ���-�����, ������ �������� � ������� ������� ������ �� ����� ���������;
- ���������� ������ ������ ��� ���� � ������������ � ����������� �����������;
- ������������� ������������� ��������� ���������� �������������� ��������.

���� - ��������� ��������������� �������� �������� ��:
- ������������ ����������� ������ ��������� � ������ ���������� ����;
- �� ��������� ������� �������������� ���������� ������������� ���������� ��� ���������� � ��� � ��������� ��������,
  ������������� ���������� ������������� ���� ����������. ������������ ������ �������.

���� - ��������� ���������� � ��������������:
- ����������� ��������� �������� �� � �� � ������ ����������� ��������, � ��� ����� � ������ ������ ��� ��;
- ��������� ��������� �������������� �������� � ��� � ���;
- ��� ������ ���� � ����������� �����, ��������������� ���������� ����, ���-������� ������������ ������,
  ������� ������������ ������� ������ ��������� �� �� ���������� ���������� �������������� ��������.

���� - ���������� ������� ������� ��������:
- �� ������ ������ ��������� ������������ ��������������� ��������� (��, ��, ����),������� ���������� ������������
  ������� ��������� ��������� ��� ���-������ �������� ������ ���� ��. � �����������, � ����� ������ �������� ��������,
  ����� �������� ������ (����������) �� ���������� ���������� ���������� �������� � ��������� �� ��� ��� ��� ���.
  ��������� ������� ��������� ��������������� ��� ������ ���;
- ��������� ����, ��� ��������� ����� ��������� ����, ������������ ��������, �������������� ������������ ������ �
  ����� ������ ���������� ������������ ��� ����������� �������� ���������.

���� - ��������� ����������� ������� ��������:
- ���������� ��������������� �� ��������� ��� �� ������ ���������� ��������, ������������� ���� � ��������� �� ��� ��� ��� ���;
- ���������  � - ���������� ������������ ��������� �� ������������ ��� ��� ��� ��� ��������� ��������������� �������� �� ���.

��� - ��������� ���������� � ������������:
- � ����������� �� ��������� ������ ���������� (��� ��� ���) ������������ ��-�������� � ����������� ���� ���������� ��
  � ������������ ���������.

�������� ��� - ������������ ������ �������� � �������������� ������� ��� 
         ���������� � ����������� �������� ����������;
- ������������ ����������� �������� ���������� ��,

���� - ��������� ���������� ��������� ���������:
- �������� �������������� ������ ��� ���������� �� � ��� ���������� ��������� ������ ����;
- �������� ��������� ���� ������ ���;
- ��������� ������ ���������� ����� ������ ��� � �������������� � ������������������ ������� ��.

��������� ��������� ������ ���:
- ���������� ��������� ���� ��� ��������� �� ���;

���� - ��������� ���������� �������� ���������:
- �� ��������� ���������� � �������������, ������� ������ ��� � ��������������� ��� ���� ��������� �������� ���������� ��
  ������ �������������� ������� �� ��������� ��� ��.

��������� ����:
- ���������� �� ���� ���������� �� �������� ��������� ���;
- ��������� �������������� ��������� ����� ��� ���� � �������� ������� �������.

��������� ���:
- ������������ ������ �� ����� ������� ���������� � ����������� ������ � ���� ��������.

��������� ����������:
- ������ �� ���� ��������� � ����������� ��������������� ���������� � ����������� �������.







*/
