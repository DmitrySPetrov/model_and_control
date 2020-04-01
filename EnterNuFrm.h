//---------------------------------------------------------------------------

#ifndef EnterNuFrmH
#define EnterNuFrmH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ComCtrls.hpp>
#include <ExtCtrls.hpp>
#include <Graphics.hpp>
#include <Buttons.hpp>
#include <Dialogs.hpp>
//---------------------------------------------------------------------------
class TEnterNuForm : public TForm
{
__published:	// IDE-managed Components
        TLabel *DataLabel;
        TLabel *Label17;
        TPageControl *NuParamTabs;
        TTabSheet *complex_tab;
        TTabSheet *orbitpar_tab;
        TGroupBox *GroupBox1;
        TLabel *Label1;
        TLabel *Label2;
        TLabel *Label3;
        TLabel *Label4;
        TLabel *Label5;
        TLabel *Label6;
        TLabel *Label7;
        TLabel *Label8;
        TEdit *b_pol_tk;
        TEdit *e_tk;
        TEdit *dol_v_tk;
        TEdit *i_tk;
        TEdit *arg_per_tk;
        TEdit *an_is_tk;
        TEdit *ball_coef_tk;
        TGroupBox *GroupBox2;
        TLabel *Label9;
        TLabel *Label10;
        TLabel *Label11;
        TLabel *Label12;
        TLabel *Label13;
        TLabel *Label14;
        TLabel *Label15;
        TLabel *Label16;
        TEdit *b_pol_iss;
        TEdit *e_iss;
        TEdit *dol_v_iss;
        TEdit *i_iss;
        TEdit *arg_per_iss;
        TEdit *an_is_iss;
        TEdit *ball_koef_iss;
        TRadioGroup *OrbitType;
        TGroupBox *CharTK;
        TGroupBox *GroupBox4;
        TLabel *Label18;
        TEdit *Jxx_tk;
        TEdit *Jyy_tk;
        TLabel *Label19;
        TLabel *Label20;
        TEdit *Jzz_tk;
        TEdit *Jxy_tk;
        TLabel *Label21;
        TLabel *Label22;
        TEdit *Jyz_tk;
        TEdit *Jzx_tk;
        TLabel *Label23;
        TLabel *Label24;
        TEdit *mass_tk;
        TLabel *Label25;
        TGroupBox *GroupBox5;
        TLabel *Label26;
        TEdit *cm_x_tk;
        TLabel *Label27;
        TLabel *Label28;
        TEdit *cm_y_tk;
        TLabel *Label29;
        TLabel *Label30;
        TEdit *cm_z_tk;
        TLabel *Label31;
        TGroupBox *CharISS;
        TLabel *Label32;
        TLabel *Label33;
        TGroupBox *GroupBox7;
        TEdit *Jxx_iss;
        TEdit *Jyy_iss;
        TEdit *Jzz_iss;
        TEdit *Jxy_iss;
        TEdit *Jyz_iss;
        TEdit *Jzx_iss;
        TEdit *mass_iss;
        TGroupBox *GroupBox8;
        TLabel *Label40;
        TLabel *Label41;
        TLabel *Label42;
        TLabel *Label43;
        TLabel *Label44;
        TLabel *Label45;
        TEdit *cm_x_iss;
        TEdit *cm_y_iss;
        TEdit *cm_z_iss;
        TGroupBox *GroupBox9;
        TGroupBox *GroupBox10;
        TRadioButton *RadioButton1;
        TRadioButton *RadioButton2;
        TLabel *Label46;
        TEdit *Edit36;
        TGroupBox *GroupBox11;
        TRadioButton *RadioButton3;
        TRadioButton *RadioButton4;
        TGroupBox *GroupBox12;
        TLabel *Label47;
        TEdit *Edit37;
        TEdit *Edit38;
        TLabel *Label48;
        TGroupBox *GroupBox13;
        TLabel *Label49;
        TLabel *Label50;
        TEdit *Edit39;
        TEdit *Edit40;
        TEdit *Edit41;
        TEdit *Edit42;
        TEdit *Edit43;
        TLabel *Label51;
        TLabel *Label52;
        TEdit *Edit44;
        TEdit *Edit45;
        TButton *Button1;
        TLabel *dfgdf;
        TEdit *tk_toplivo;
        TLabel *Label54;
        TTabSheet *ParAndOr;
        TGroupBox *GroupBox20;
        TEdit *ro_init;
        TLabel *Label80;
        TEdit *ro_dot_init;
        TLabel *Label82;
        TEdit *Edit61;
        TLabel *Label83;
        TEdit *Edit62;
        TLabel *Label84;
        TEdit *Edit63;
        TLabel *Label85;
        TEdit *Edit64;
        TLabel *Label86;
        TRadioButton *RadioButton5;
        TRadioButton *RadioButton6;
        TRadioButton *RadioButton7;
        TRadioButton *RadioButton8;
        TRadioButton *docking_pr;
        TRadioButton *redock10_pr;
        TGroupBox *GroupBox21;
        TRadioButton *sk_tk_tp;
        TRadioButton *sk_tk_osk;
        TRadioButton *sk_tk_vsk;
        TGroupBox *GroupBox23;
        TLabel *Label90;
        TLabel *Label91;
        TLabel *Label92;
        TEdit *w_x_tk;
        TEdit *w_y_tk;
        TEdit *w_z_tk;
        TGroupBox *GroupBox24;
        TRadioButton *sk_iss_tp;
        TRadioButton *sk_iss_osk;
        TRadioButton *sk_iss_vsk;
        TGroupBox *GroupBox25;
        TLabel *Label93;
        TLabel *Label94;
        TLabel *Label95;
        TEdit *w_x_iss;
        TEdit *w_y_iss;
        TEdit *w_z_iss;
        TGroupBox *GroupBox26;
        TLabel *Label96;
        TLabel *Label97;
        TLabel *Label98;
        TEdit *ang_psi_iss;
        TEdit *ang_thetta_iss;
        TEdit *ang_gamma_iss;
        TGroupBox *GroupBox27;
        TGroupBox *GroupBox22;
        TLabel *Label87;
        TLabel *Label88;
        TLabel *Label89;
        TEdit *ang_psi_tk;
        TEdit *ang_thetta_tk;
        TEdit *ang_gamma_tk;
        TImage *Image1;
        TImage *Image2;
        TImage *Image3;
        TImage *Image4;
        TImage *Image5;
        TImage *Image6;
        TImage *Image7;
        TImage *Image8;
        TImage *Image9;
        TImage *Image10;
        TImage *Image11;
        TImage *Image12;
        TImage *Image13;
        TImage *Image14;
        TImage *Image15;
        TImage *Image16;
        TImage *Image17;
        TImage *Image18;
        TLabel *Label79;
        TRadioButton *r_st_free;
        TRadioButton *r_st_tp;
        TRadioButton *r_st_osk;
        TRadioButton *r_st_wconst;
        TCheckBox *CheckBox1;
        TCheckBox *CheckBox2;
        TGroupBox *GroupBox28;
        TCheckBox *CheckBox3;
        TCheckBox *CheckBox4;
        TCheckBox *CheckBox5;
        TCheckBox *CheckBox6;
        TGroupBox *GroupBox29;
        TEdit *Edit16;
        TLabel *Label81;
        TLabel *Label99;
        TEdit *Edit77;
        TGroupBox *GroupBox30;
        TComboBox *docking_uz;
        TGroupBox *GroupBox31;
        TComboBox *dk_su;
        TGroupBox *GroupBox32;
        TLabel *Label100;
        TEdit *Edit78;
        TLabel *Label101;
        TEdit *Edit79;
        TLabel *Label102;
        TLabel *Label103;
        TLabel *Label104;
        TLabel *Label105;
        TLabel *Label106;
        TLabel *Label107;
        TLabel *Label108;
        TLabel *Label109;
        TImage *Image19;
        TImage *Image20;
        TImage *Image21;
        TOpenDialog *LoadNuFromFile;
        TBitBtn *LoadFromFileBtn;
        TLabel *Label34;
        TLabel *Label35;
        TLabel *Label36;
        TLabel *Label37;
        TLabel *Label38;
        TLabel *Label39;
        TGroupBox *GroupBox16;
        TLabel *Label61;
        TLabel *Label62;
        TLabel *Label63;
        TLabel *Label64;
        TLabel *Label65;
        TLabel *Label66;
        TEdit *iss_pos_x;
        TEdit *iss_pos_y;
        TEdit *iss_pos_z;
        TGroupBox *GroupBox15;
        TLabel *Label55;
        TLabel *Label56;
        TLabel *Label57;
        TLabel *Label58;
        TLabel *Label59;
        TLabel *Label60;
        TEdit *iss_vel_x;
        TEdit *iss_vel_y;
        TEdit *iss_vel_z;
        TGroupBox *GroupBox19;
        TLabel *Label73;
        TLabel *Label74;
        TLabel *Label75;
        TLabel *Label76;
        TLabel *Label77;
        TLabel *Label78;
        TEdit *tk_pos_x;
        TEdit *tk_pos_y;
        TEdit *tk_pos_z;
        TGroupBox *GroupBox18;
        TLabel *Label67;
        TLabel *Label68;
        TLabel *Label69;
        TLabel *Label70;
        TLabel *Label71;
        TLabel *Label72;
        TEdit *tk_vel_x;
        TEdit *tk_vel_y;
        TEdit *tk_vel_z;
        TBitBtn *BitBtn1;
        TSaveDialog *SaveNuToFile;
        TLabel *Label53;
        TTabSheet *TabSheet1;
        TRadioButton *reg_00;
        TRadioButton *reg_01;
        TRadioButton *reg_02;
        TRadioButton *reg_03;
        TRadioButton *reg_04;
        TRadioButton *reg_05;
        TDateTimePicker *SetModelTimePicker;
        TDateTimePicker *SetModelDatePicker;
        TGroupBox *GroupBox3;
        TRadioButton *sm_dp_so;
        TRadioButton *sm_dp_mlm;
        TGroupBox *GroupBox6;
        TRadioButton *obj_tpk;
        TRadioButton *obj_tgk;
        TRadioButton *obj_mlm;
        TGroupBox *mode;
        TRadioButton *avtsbl_pr;
        TRadioButton *rodk_pr;
        TRadioButton *roak_pr;
        void __fastcall LoadFromFileBtnClick(TObject *Sender);
        void __fastcall BitBtn1Click(TObject *Sender);
        //void __fastcall TabControl1Change(TObject *Sender);
        //void __fastcall NuParamTabsChange(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TEnterNuForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TEnterNuForm *EnterNuForm;
//---------------------------------------------------------------------------
#endif
