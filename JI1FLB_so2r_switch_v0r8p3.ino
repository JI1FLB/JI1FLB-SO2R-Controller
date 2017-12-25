/*
JI1FLB SO2Rコントローラ Copyright (C) JI1FLB Seiichi Tanaka 2017/06/10
Licens: This sketch is a freeware based on GNU GENERAL PUBLIC LICENS Version 3.
Last Modify:2017/06/10 18:24
Version：0版
Revise ：8版
Patch ： 3回目
Protocol ： OTRSP http://www.k1xm.org/OTRSP/OTRSP_Protocol.pdf
History of Features which is appending.：
	・マイク切替
	・PTT切替
	・ヘッドホーン切替
	・CWポート切替
	・Ant Band情報(AUX1対応のみ)		v0r2p6
	・ボイスメモリ対応機能の検証		v0r6p1
	・QueueListのQUEUEを利用廃止		v0r6p1
	・Query系に？のみ受信時の処理		v0r6p1
	・手動切替（強制リグロック機能）	v0r6p22
	・プッシュボタン動作でリグのロックをOn/Offする
			注：外部割込み(Int.0,Int.1)を利用し、プッシュボタン動作でリグのロックをOn/Offする機能はFS
	・OTRSP buffer Clear feature	V0r6p23
		OTRSP関連のSerial通信のバッファと、OTRSPcommandバッファをクリアする機能
	・I2C　LCDディスプレー表示機能		v0r7p0
	・手動切替モード(Face switch feature)	V0r8p0

*/


/* Histoy of Modify

yyyy/mm/dd
	ヘッドホーン動作マトリックスに合わせて、ディジタルポート動作を変更
2014年9月2日 8:28:03
	一部変数名のアンダバー削除、LockingHandling()の追加
2014年9月14日17:17:12
	ソフトウェア構造を大幅に変更
	各ペリフェイラルからの情報を受け取るルーチンに変更
2016/12/10 v0r8p0 
	Face Switch featuresのソフトウェア変更
	ArudinoUNOのポートアサインメントの整理
2017年6月10日18:17:43 v0r8p3
	command_handling(),aux1_output_handling(),LcdDisplayLockedSo2r(),LcdDisplaySo2r()のダイナミックステップ改善

*/


/* N1MM OTRSP　キャプチャ結果　2013/11/09

\ key	RXx
@ key	RXxS
Pause key	TXx RXx or TXx RXxS
Ctrl+Left arrow  key	Leftに設定したリグ　TXx RXx or TXx RXxS
Ctrl+Right arrow key	Rightに設定したリグ　TXx RXx or TXx RXxS
Band change
	antenna tabに設定されている場合		AUXx0n
	antenna tabに設定されていない場合	AUXx-01
判ったこと
	受信時のリバース機能はサポートされいない模様
	AUXは　AUXxnnでAntenna Tabのコードが一桁の場合、nnの二けた目を0でパティングする。
	Ctrl+PageUP/PageDown 時は、バンド情報が即、通知される。
	Rigでバンドチェンジした場合には、バンド情報がSO2Rデバイスに通知されるまで、少し時間がかかる。

*/

//------------------------------------------------------------------

#include <Wire.h>			// Arduino IDE のI2Cライブラリ
#include <LcdCore.h>		// LCDコアライブラリ
#include <LCD_ACM1602NI.h>	// 秋月I2C液晶用のI/Oライブラリ
LCD_ACM1602NI lcd(0xa0);	// 0xa0はI2Cアドレス

//-------------------------------------------------------------------
//                   0123456789012345
#define message00	"JI1FLB SO2R Box "
#define message01	"<Rig- L><Rig- R>"
#define message02	"Left      Locked"
#define message03	"Right     Locked"
#define message04	"--------        "
#define message05	"        --------"
#define message06	"--------"
#define message07	"Rx Tx   --------"
#define message08	"Rx  "
#define message09	"RxS "
#define message10	"RxR "
#define message11	"Tx  "
#define message12	"-------- Rx Tx  "
#define message13	"Rx Tx  Rx       "
#define message14	"Rx     Rx   Tx  "
#define message15	" Locked --------"
#define message16	"-------  Locked "
#define message17	"----"
#define message18	"Left  ST  Locked"
#define message19	"Right ST  Locked"


//-------------------------------------------------------------------------------
// 変数の定義
//


	int		Rig1PushSwStatus ;
	int		Rig2PushSwStatus ;
	int		StereoPushSwStatus ;

	unsigned	long	GuardTime= 250 ;
	unsigned	long	NowTimeRig1 ;
	unsigned	long	NowTimeRig2 ;
	unsigned	long	NowTimeStereo ;
	unsigned	long	OldTimeRig1 ;
	unsigned	long	OldTimeRig2 ;
	unsigned	long	OldTimeStereo ;
	unsigned	long	TimeRig1 ;
	unsigned	long	TimeRig2 ;
	unsigned	long	TimeStereo ;

	boolean	OccuredLocking ;
	boolean	Rig1IsLocked ;
	boolean	Rig2IsLocked ;
	boolean	StereoIsLocked ;
	boolean	RigLockedStatus ;
	boolean	OldRigLockedStatus ;


// for generel Constants
	String	OTRSPcommand ;
	String	RxStatus ;
	String	TxStatus ;
	String	OldRxStatus ;
	String	OldTxStatus ;
	String	AuxStatus ;
	String	DVKStatus ;
	boolean	Recived ;

// Definition of constants and pin numbers
// Constants for Locked Rig

	const	int		Rig1PushSwPort = 2 ;	// Constants for Locked Rig
	const	int		Rig2PushSwPort = 3 ;	// Constants for Locked Rig
	const	int		StereoPushSwPort = 1 ;	// Constants for Locked Rig

	const	int		RX_cont_RL1 = 4 ;
	const	int		RX_cont_RL2 = 5 ;
	const	int		TX_sw_RL = 6 ;
	const	int		CW_padle = 7 ;

	const	int		AUX0_1 = 8 ;
	const	int		AUX0_2  = 9;
	const	int		AUX0_4  = 10 ;
	const	int		AUX0_8  = 11 ;

	const	int		Reverse = 12 ;
	const	int		Stereo = 13 ;
	
	const	int		DVKm1 = 8 ;
	const	int		DVKm2 = 9 ;
	const	int		DVKm3 = 10 ;
	const	int		DVKm4 = 11 ;
	const	int		OTRSPlength = 17;
	
// Constants for FW & System Name
	const	String	FW	 =	 "0.8.3" ;
	const	String	SO2R_name = "JI1FLB SO2R Controler" ;


//--------------------------------------
//
// the setup routine run
//
//--------------------------------------



void setup() {

  // initialize the digital pin as an output.
	
	pinMode( RX_cont_RL1 , OUTPUT ) ;
	pinMode( RX_cont_RL2 , OUTPUT ) ;
	pinMode( TX_sw_RL , OUTPUT ) ;
	pinMode( CW_padle , OUTPUT ) ;
//	pinMode( AUX0_1 , OUTPUT ) ;
//	pinMode( AUX0_2 , OUTPUT ) ;
//	pinMode( AUX0_4 , OUTPUT ) ;
//	pinMode( AUX0_8 , OUTPUT ) ;
	pinMode( Stereo , OUTPUT ) ;
	pinMode( Reverse , OUTPUT ) ;
//	pinMode( DVKm1 , OUTPUT ) ;
//	pinMode( DVKm2 , OUTPUT ) ;
//	pinMode( DVKm3 , OUTPUT ) ;
//	pinMode( DVKm4 , OUTPUT ) ;

//	for Locked Rig Switching
	pinMode( Rig1PushSwPort , INPUT_PULLUP );
	pinMode( Rig2PushSwPort , INPUT_PULLUP ); 
	pinMode( StereoPushSwPort , INPUT_PULLUP ); 

//	for Arduino UNO Serial port

	Serial.begin( 9600 ) ;
	Serial.println( SO2R_name ) ;
	
	OTRSPcommand = "" ;
	Recived = false ;
	RxStatus = "RX1" ;
	TxStatus = "TX1" ;
	OldRxStatus = RxStatus ;
	OldTxStatus = TxStatus ;

	RX1_on() ; 
	TX1_on() ; 


//	for Locked Rig
	NowTimeRig1 = 0 ;
	NowTimeRig2 = 0 ;
	OldTimeRig1 = 0 ;
	OldTimeRig2 = 0 ;
	TimeRig1 =  0 ;
	TimeRig2 =  0 ;
	NowTimeStereo =  0  ;
	OldTimeStereo =  0  ;
	TimeStereo =  0 ;
	Rig1IsLocked = false ;
	Rig2IsLocked = false ;
	StereoIsLocked = false ;
	OccuredLocking = false ;
	Rig1PushSwStatus = HIGH ;
	Rig2PushSwStatus = HIGH ;
	StereoPushSwStatus = HIGH ;
	OldRigLockedStatus = false ;
	RigLockedStatus = false ;

// Initialization of LCD display
	lcd.begin(16, 2);
	lcd.clear() ;
	lcd.home() ;
	lcd.print( message00 );
	lcd.setCursor(0,1) ;
	lcd.print( " FW = ");
	lcd.print(FW );
	delay( 3000 ) ;
	lcd.clear() ;

}


//----------------------------------------------------
//
//
//
//----------------------------------------------------

void LockCheckRig1(){
/*
  Specification
  Rig 1 push button on and Rig 1 is Lock
  If you press the same button unlock
*/
	
/*
	仕様
		Rig1プッシュボタン　オンで　Rig1がLock
		同じボタンを押した場合にはロックを解除
*/

	Rig1PushSwStatus = digitalRead ( Rig1PushSwPort ) ;
	
	if( Rig1PushSwStatus == LOW ){
		NowTimeRig1 = millis() ;
		TimeRig1 = NowTimeRig1 - OldTimeRig1 ;
		if( TimeRig1 > GuardTime ) {
			Rig1IsLocked = !Rig1IsLocked ;
			Rig2IsLocked = false ;
			OldTimeRig1 = NowTimeRig1 ;
//			OldRxStatus = RxStatus ;
//			OldTxStatus = TxStatus ;
		}
	}
	if( Rig1IsLocked ) {
		RxStatus = "RX1" ;
		TxStatus = "TX1" ;
		RigLockedStatus = true ;
		OldRigLockedStatus = RigLockedStatus ;
	} else if( !Rig2IsLocked ){
		RigLockedStatus = false ;
		OldRigLockedStatus = RigLockedStatus ;
	}
}


void LockCheckRig2(){
/*
  Specification
    Rig 2 push button on and Rig 2 is Lock
    If you press the same button,a locked status transfer to a unlocked status.
*/
	
/*
	仕様
		Rig2プッシュボタン　オンで　Rig2がLock
		同じボタンを押した場合にはロックを解除
*/

	Rig2PushSwStatus = digitalRead ( Rig2PushSwPort ) ;
	
	if( Rig2PushSwStatus == LOW ){
		NowTimeRig2 = millis() ;
		TimeRig2 = NowTimeRig2 - OldTimeRig2 ;
		if( TimeRig2 > GuardTime ) {
			Rig2IsLocked = !Rig2IsLocked ;
			Rig1IsLocked = false ;
			OldTimeRig2 = NowTimeRig2 ;
//			OldRxStatus = RxStatus ;
//			OldTxStatus = TxStatus ;
		}
	}
	if( Rig2IsLocked ) {
		RxStatus = "RX2" ;
		TxStatus = "TX2" ;
		RigLockedStatus = true ;
		OldRigLockedStatus = RigLockedStatus ;
	} else if( !Rig1IsLocked ) {
		RigLockedStatus = false ;
		OldRigLockedStatus = RigLockedStatus ;
	}
}


void LockCheckStereo(){
/*
Specification
Operating conditions are force switch state (RigLockedStatus is true)
Stereo state with Stereo push button on
If you press the same button unlock
*/
	
/*
	仕様
		動作条件は、フォーススイッチ状態であること( RigLockedStatusがtrue )
		Stereoプッシュボタン　オンで　Stereo状態k
		同じボタンを押した場合にはロックを解除
*/

	StereoPushSwStatus = digitalRead ( StereoPushSwPort ) ;
	if ( RigLockedStatus ) {
		if( StereoPushSwStatus == LOW ){
			NowTimeStereo = millis() ;
			TimeStereo = NowTimeStereo - OldTimeStereo ;
			if( TimeStereo > GuardTime ) {
				StereoIsLocked = !StereoIsLocked ;
				OldTimeStereo = NowTimeStereo ;
			}
		}

	    // Change for mono to Stereo
		if( StereoIsLocked ) {
			if( Rig1IsLocked ) {
				if(RxStatus == "RX1"){
					RxStatus = "RX1S" ;
				} 
			}
			if( Rig2IsLocked ) {
				if(RxStatus == "RX2"){
					RxStatus = "RX2S" ;
				}
			}
		}
			
			
	    // change from Stereo to mono
		if( !StereoIsLocked ) {
			if(RxStatus == "RX1S"){
				RxStatus = "RX1" ;
			} 
			if(RxStatus == "RX2S"){
	 			RxStatus = "RX2" ;           
			}
			
		}
	}
}


// 	OTRSP SO2R Command Reciveing control function

void command_reciveing() {

// Command messege structure: QueryPrefix + Keyword + Item + Value + CR(hex 0D  Decimal 13)
// COMMAND reciveing

//	while ( !Recived ){
		if ( Serial.available() > 0 ) {
			byte buffer=Serial.read() ;
			if (OTRSPcommand.length() > 17 ) {				// Checking Recived Command Lengths 
				OTRSPcommand = "" ;
				Recived = false ;
//				break ;
				}
			if (buffer==0x0d) {				// When recived a CR code, start to analyze recived commnads.
				Recived = true ;
//				break ;
			}
			OTRSPcommand +=(char) buffer ;
		}
//	}
}

void OtrspBufferClear() {

	if (Serial.available() > 0) {
		for ( int i = 0 ; i < Serial.available() ; i++){
			byte TrushRecivedData = Serial.read() ;
		}
	}
	
//	for ( int i = 1 ; i < OTRSPlength ; i++){
//		if (Serial.available() > 0) {
//				byte TrushRecivedData = Serial.read() ;
//		}
//	}
	
	OTRSPcommand = "" ;
}


void command_handling(){

		if (OTRSPcommand.startsWith( "TX1" ) ) {
			TxStatus = "TX1" ;
			return ;
			} 
		if (OTRSPcommand.startsWith( "TX2" ) ) {
			TxStatus = "TX2" ;
			return ;
		}

		if (OTRSPcommand.startsWith( "RX1" ) ) {
			RxStatus = "RX1" ;
			if (OTRSPcommand.startsWith( "RX1S" ) ) {
				RxStatus = "RX1S" ;
				return ;
			}
			if (OTRSPcommand.startsWith( "RX1R" ) ) {
				RxStatus = "RX1R" ;
				return ;
			}
		}

		if ( OTRSPcommand.startsWith( "RX2" )   ) {
			RxStatus = "RX2" ;
			if  ( OTRSPcommand.startsWith( "RX2S" )  ) {
				RxStatus = "RX2S" ;
				return ;
			} 
			if  ( OTRSPcommand.startsWith( "RX2R" )  ) {
				RxStatus = "RX2R" ;
				return ;
			}
		}

		if (OTRSPcommand.startsWith( "AUX" ) ) {
			AuxStatus = OTRSPcommand ;
			return ;
		}

		if ( OTRSPcommand.startsWith( "KP1" )   ) {
			DVKStatus = "KP1" ;
			return ;
		}
		if ( OTRSPcommand.startsWith( "KP2" )   ) {
			DVKStatus = "KP2" ; 
			return ;
		}
		if ( OTRSPcommand.startsWith( "KP3" )   ) {
			DVKStatus = "KP3" ;
			return ;
		}
		if ( OTRSPcommand.startsWith( "KP4" )   ) {
			DVKStatus = "KP4" ; 
			return ;
		}


	//
	//		analyzing QUERY's command, and handling.
	//
	
	if ( OTRSPcommand.charAt(0) == '?' ){    
		if (OTRSPcommand.startsWith("?NAME")) {
			Serial.println ( SO2R_name) ; 
			return ;
		}
		else if (OTRSPcommand.startsWith("?FW")) {
			Serial.println ( FW ) ; 
			return ;
		}
		else if (OTRSPcommand.startsWith("?RX")) {
			Serial.println ( RxStatus ) ;
			return ;
		} 
		else if (OTRSPcommand.startsWith("?TX")) {
			Serial.println ( TxStatus ) ;
			return ;
		}
		else if (OTRSPcommand.startsWith("?KP")) {
//			Serial.println ( DVKStatus ) ;
			Serial.println ( "KP" ) ;
			return ;
		}
		// If recived only '?" query, this device reply also '?".
		else if (OTRSPcommand.length() == 2 ) {
			Serial.println ( "?" ) ;
			return ;
		}
		else {
			// When recived an unrecognized command Query,reply to send same command.
			Serial.println ( OTRSPcommand.substring(1) ) ;
			return ;
		}
	}
}

// ----------------------------------------	
//	device control functions
//-----------------------------------------	

void	DeviceControl(){

		if (RxStatus == "RX1" ){
			RX1_on() ;
					}
		else if (RxStatus == "RX1S" ){
			RX1S_on() ;
		}
		else if (RxStatus == "RX1R" ){
			RX1R_on() ;
		}
		else if (RxStatus == "RX2" ){
			RX2_on() ;
		}
		else if (RxStatus == "RX2S" ){
			RX2S_on() ;
		}
		else if (RxStatus == "RX2R" ){
			RX2R_on() ;
		}
	
		if (TxStatus == "TX1" ){
			TX1_on() ;
		}
		else if (TxStatus == "TX2" ){
			TX2_on() ;
		}
			
		if (AuxStatus.startsWith( "AUX" ) ) {
			aux1_output_handling() ;
		}

		if (DVKStatus == "KP1" ){
			KP1_OnOff() ;
			DVKStatus = "" ; 
		}

		else if (DVKStatus == "KP2" ){
			KP2_OnOff() ;
			DVKStatus = "" ; 
		}

		else if (DVKStatus == "KP3" ){
			KP3_OnOff() ;
			DVKStatus = "" ; 
		}
			
		else if (DVKStatus == "KP4" ){
			KP4_OnOff() ;
			DVKStatus = "" ; 
		}

		if ( RigLockedStatus ) {
			Locked();
		} else {
			UnLocked();
		}
}

//
//		SO2R device control functions
//

void Locked() {
	digitalWrite(CW_padle, HIGH) ;
}

void UnLocked() {
	digitalWrite(CW_padle, LOW) ;
}

void RX1_on() {
	digitalWrite(RX_cont_RL1, LOW) ;
	digitalWrite(RX_cont_RL2, LOW) ;
	digitalWrite(Stereo, LOW) ;
//	digitalWrite(Reverse , LOW) ;
	} 

void RX1S_on(){
	digitalWrite(RX_cont_RL1, HIGH) ;
	digitalWrite(RX_cont_RL2, LOW) ;
	digitalWrite(Stereo , HIGH) ;
//	digitalWrite(Reverse , LOW) ;
	}

void RX1R_on(){
	digitalWrite(RX_cont_RL1, LOW) ; 
	digitalWrite(RX_cont_RL2, HIGH) ; 
	digitalWrite(Stereo , LOW) ; 
//	digitalWrite(Reverse , HIGH) ; 
	}

void RX2_on(){
	digitalWrite(RX_cont_RL1, HIGH) ; 
	digitalWrite(RX_cont_RL2, HIGH) ;
	digitalWrite(Stereo , LOW) ; 
//	digitalWrite(Reverse , LOW) ;
	}

void RX2S_on(){
	digitalWrite(RX_cont_RL1, HIGH)  ;
	digitalWrite(RX_cont_RL2, LOW) ;
	digitalWrite(Stereo , HIGH) ; 
//	digitalWrite(Reverse , LOW) ;
	}

void RX2R_on(){
	digitalWrite(RX_cont_RL1, LOW) ;
	digitalWrite(RX_cont_RL2, HIGH) ;
	digitalWrite(Stereo , LOW) ; 
//	digitalWrite(Reverse , HIGH) ;
	}

void TX1_on() {
	digitalWrite(TX_sw_RL, LOW ) ;
	} 

void TX2_on(){
	digitalWrite(TX_sw_RL, HIGH ) ;
	}
	
void KP1_OnOff(){
	digitalWrite(DVKm1, HIGH ) ;
	delay(100) ;
	digitalWrite(DVKm1, LOW ) ;
	}

void KP2_OnOff(){
	digitalWrite(DVKm2, HIGH ) ;
	delay(100) ;
	digitalWrite(DVKm2, LOW ) ;
	}

void KP3_OnOff(){
	digitalWrite(DVKm3, HIGH ) ;
	delay(100) ;
	digitalWrite(DVKm3, LOW ) ;
	}

void KP4_OnOff(){
	digitalWrite(DVKm4, HIGH ) ;
	delay(100) ;
	digitalWrite(DVKm4, LOW ) ;
	}


void aux1_output_handling() {
/*
Convert Anttena tab code from N1MM to BCD to output Aux output (4 bits output)
AUX FORMAT AUXxnn at N1MM. Pad the space of 0 with 0.
Example) When antenna tab code is '1', decode result is expressed by 4 bit output in the band of '01'
By changing the statement of the IF statement, various output matching the code of Antenna Tab is possible.
*/
// N1MMからのAnttena tab codeをBCDに変換してAux出力（4bits output）
// AUX FORMAT  AUXxnn at N1MM. nn の空きを0でpading.例）antenna tab code '1'の場合、"01"となる
// バンドでデコード結果を4ビットの出力で表す
// IF文のステートメントを変更することで、Antenna Tabのコードに合わせたいろいろな出力が可能となる。

	if ( AuxStatus.substring(4,6) == "00" ){    
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , LOW ) ; 
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "01" ){    
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , LOW ) ; 
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "02" ){    
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , LOW ) ;
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "03" ){    
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , LOW ) ; 
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "04" ){ 
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , HIGH ) ;
		digitalWrite(AUX0_8 , LOW ) ;  
		return ;
		}
	if ( AuxStatus.substring(4,6) == "05" ){ 
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , HIGH ) ;
		digitalWrite(AUX0_8 , LOW ) ;  
		}
	if ( AuxStatus.substring(4,6) == "06" ){ 
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , HIGH ) ;
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "07" ){ 
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , HIGH ) ;
		digitalWrite(AUX0_8 , LOW ) ; 
		return ;
		}
	if ( AuxStatus.substring(4,6) == "08" ){
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , LOW ) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "09" ){    
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , LOW )  ; 
		digitalWrite(AUX0_4 , LOW ) ;
		digitalWrite(AUX0_8 , HIGH ) ; 
		}
	if ( AuxStatus.substring(4,6) == "10" ){    
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , HIGH )  ;
		digitalWrite(AUX0_4 , LOW ) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "11" ){    
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , LOW ) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "12" ){
		digitalWrite(AUX0_1 , LOW ) ;
		digitalWrite(AUX0_2 , LOW  )  ; 
		digitalWrite(AUX0_4 , HIGH ) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "13" ){
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , LOW  )  ; 
		digitalWrite(AUX0_4 , HIGH  ) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "14" ){
		digitalWrite(AUX0_1 , LOW  ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , HIGH) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
	if ( AuxStatus.substring(4,6) == "15" ){
		digitalWrite(AUX0_1 , HIGH ) ;
		digitalWrite(AUX0_2 , HIGH )  ; 
		digitalWrite(AUX0_4 , HIGH) ;
		digitalWrite(AUX0_8 , HIGH ) ;
		return ;
		}
}


// Forced SwitchのLCDディスプレ表示設定機能
//

void LcdDisplayLockedSo2r() {

	lcd.setCursor(0,0) ;
	lcd.print( message01 ) ;

	lcd.setCursor(0,1) ;
	
	if ( Rig1IsLocked && !StereoIsLocked) {
		lcd.setCursor(0,1) ;
		lcd.print(message02) ;
		return ;
	}
	
	if ( Rig2IsLocked && !StereoIsLocked ) {
		lcd.setCursor(0,1) ;
		lcd.print( message03 ) ;
		return ;
	}

	if ( Rig1IsLocked && StereoIsLocked ) {
		lcd.setCursor(0,1) ;
		lcd.print(message18) ;
		return ;
	}
	
	if ( Rig2IsLocked && StereoIsLocked ) {
		lcd.setCursor(0,1) ;
		lcd.print( message19 ) ;
		return ;
	}
}
	
//
//   In forced mode Switch phase, LCD display setting features
//

void LcdDisplaySo2r() {

 
	lcd.setCursor(0,0) ;
	lcd.print( message01 ) ;
	lcd.setCursor(0,1) ;
	
		if (TxStatus == "TX1" ){
		lcd.setCursor(4,1) ;
		lcd.print( message11 ) ;
		lcd.setCursor(12,1) ;
		lcd.print( message17 ) ;
	}
	if (TxStatus == "TX2" ){
		lcd.setCursor(12,1) ;
		lcd.print( message11 ) ;
		lcd.setCursor(4,1) ;
		lcd.print( message17 ) ;
	}
	
	if ( RxStatus == "RX1" ) {
		lcd.setCursor(0,1) ;
		lcd.print( message08 ) ;
		lcd.setCursor(8,1) ;
		lcd.print( message17 ) ;
		return ;
	}
	

	if ( RxStatus == "RX2" ) {
		lcd.setCursor(8,1) ;
		lcd.print( message08 ) ;
		lcd.setCursor(0,1) ;
		lcd.print( message17 ) ;
		return ;
	}


	if ( RxStatus == "RX1S" ) {
		lcd.setCursor(0,1) ;
		lcd.print( message09 ) ;
		lcd.setCursor(8,1) ;
		lcd.print( message09 ) ;
		return ;
		
	}
	

	if ( RxStatus == "RX2S" ) {
		lcd.setCursor(0,1) ;
		lcd.print( message09 ) ;
		lcd.setCursor(8,1) ;
		lcd.print( message09 ) ;
		return ;
	}
	
	if ( RxStatus == "RX1R" ) {
		lcd.setCursor(0,1) ;
		lcd.print( message10 ) ;
		lcd.setCursor(8,1) ;
		lcd.print( message10 ) ;
		return ;
	}
	
	if ( RxStatus == "RX2R" ) {
		lcd.setCursor(0,1) ;
		lcd.print( message10 ) ;
		lcd.setCursor(8,1) ;
		lcd.print( message10 ) ;
		return ;
	}

}


//----------------------------------------------------
//
//	Main function
//
//----------------------------------------------------

void loop() {
	
//  Scan Locked Switches
	
	LockCheckRig1() ;
	LockCheckRig2() ;
	LockCheckStereo() ;

//    Locked Manual mode
	
	if ( RigLockedStatus ) {
		DeviceControl() ;
		LcdDisplayLockedSo2r() ; 
		OtrspBufferClear() ;
	}
	
//   OTRSTP protocol mode
	
	if ( !RigLockedStatus ) {
		//DeviceControl() ;
		LcdDisplaySo2r() ;
		
		command_reciveing() ;	// When reciveing command, executing the below function.
		if ( Recived ){
			command_handling() ;
			DeviceControl() ;
			LcdDisplaySo2r() ;
			OTRSPcommand = "" ;
			Recived = false ;
		}
	}

}


/*


GNU GENERAL PUBLIC LICENSE
Version 3, 29 June 2007
Copyright © 2007 Free Software Foundation, Inc. <http://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.
Preamble
The GNU General Public License is a free, copyleft license for software and other kinds of works.
The licenses for most software and other practical works are designed to take away your freedom to share and change the works. By contrast, the GNU General Public License is intended to guarantee your freedom to share and change all versions of a program--to make sure it remains free software for all its users. We, the Free Software Foundation, use the GNU General Public License for most of our software; it applies also to any other work released this way by its authors. You can apply it to your programs, too.
When we speak of free software, we are referring to freedom, not price. Our General Public Licenses are designed to make sure that you have the freedom to distribute copies of free software (and charge for them if you wish), that you receive source code or can get it if you want it, that you can change the software or use pieces of it in new free programs, and that you know you can do these things.
To protect your rights, we need to prevent others from denying you these rights or asking you to surrender the rights. Therefore, you have certain responsibilities if you distribute copies of the software, or if you modify it: responsibilities to respect the freedom of others.
For example, if you distribute copies of such a program, whether gratis or for a fee, you must pass on to the recipients the same freedoms that you received. You must make sure that they, too, receive or can get the source code. And you must show them these terms so they know their rights.
Developers that use the GNU GPL protect your rights with two steps: (1) assert copyright on the software, and (2) offer you this License giving you legal permission to copy, distribute and/or modify it.
For the developers' and authors' protection, the GPL clearly explains that there is no warranty for this free software. For both users' and authors' sake, the GPL requires that modified versions be marked as changed, so that their problems will not be attributed erroneously to authors of previous versions.
Some devices are designed to deny users access to install or run modified versions of the software inside them, although the manufacturer can do so. This is fundamentally incompatible with the aim of protecting users' freedom to change the software. The systematic pattern of such abuse occurs in the area of products for individuals to use, which is precisely where it is most unacceptable. Therefore, we have designed this version of the GPL to prohibit the practice for those products. If such problems arise substantially in other domains, we stand ready to extend this provision to those domains in future versions of the GPL, as needed to protect the freedom of users.
Finally, every program is threatened constantly by software patents. States should not allow patents to restrict development and use of software on general-purpose computers, but in those that do, we wish to avoid the special danger that patents applied to a free program could make it effectively proprietary. To prevent this, the GPL assures that patents cannot be used to render the program non-free.
The precise terms and conditions for copying, distribution and modification follow.
TERMS AND CONDITIONS
0. Definitions.
“This License” refers to version 3 of the GNU General Public License.
“Copyright” also means copyright-like laws that apply to other kinds of works, such as semiconductor masks.
“The Program” refers to any copyrightable work licensed under this License. Each licensee is addressed as “you”. “Licensees” and “recipients” may be individuals or organizations.
To “modify” a work means to copy from or adapt all or part of the work in a fashion requiring copyright permission, other than the making of an exact copy. The resulting work is called a “modified version” of the earlier work or a work “based on” the earlier work.
A “covered work” means either the unmodified Program or a work based on the Program.
To “propagate” a work means to do anything with it that, without permission, would make you directly or secondarily liable for infringement under applicable copyright law, except executing it on a computer or modifying a private copy. Propagation includes copying, distribution (with or without modification), making available to the public, and in some countries other activities as well.
To “convey” a work means any kind of propagation that enables other parties to make or receive copies. Mere interaction with a user through a computer network, with no transfer of a copy, is not conveying.
An interactive user interface displays “Appropriate Legal Notices” to the extent that it includes a convenient and prominently visible feature that (1) displays an appropriate copyright notice, and (2) tells the user that there is no warranty for the work (except to the extent that warranties are provided), that licensees may convey the work under this License, and how to view a copy of this License. If the interface presents a list of user commands or options, such as a menu, a prominent item in the list meets this criterion.
1. Source Code.
The “source code” for a work means the preferred form of the work for making modifications to it. “Object code” means any non-source form of a work.
A “Standard Interface” means an interface that either is an official standard defined by a recognized standards body, or, in the case of interfaces specified for a particular programming language, one that is widely used among developers working in that language.
The “System Libraries” of an executable work include anything, other than the work as a whole, that (a) is included in the normal form of packaging a Major Component, but which is not part of that Major Component, and (b) serves only to enable use of the work with that Major Component, or to implement a Standard Interface for which an implementation is available to the public in source code form. A “Major Component”, in this context, means a major essential component (kernel, window system, and so on) of the specific operating system (if any) on which the executable work runs, or a compiler used to produce the work, or an object code interpreter used to run it.
The “Corresponding Source” for a work in object code form means all the source code needed to generate, install, and (for an executable work) run the object code and to modify the work, including scripts to control those activities. However, it does not include the work's System Libraries, or general-purpose tools or generally available free programs which are used unmodified in performing those activities but which are not part of the work. For example, Corresponding Source includes interface definition files associated with source files for the work, and the source code for shared libraries and dynamically linked subprograms that the work is specifically designed to require, such as by intimate data communication or control flow between those subprograms and other parts of the work.
The Corresponding Source need not include anything that users can regenerate automatically from other parts of the Corresponding Source.
The Corresponding Source for a work in source code form is that same work.
2. Basic Permissions.
All rights granted under this License are granted for the term of copyright on the Program, and are irrevocable provided the stated conditions are met. This License explicitly affirms your unlimited permission to run the unmodified Program. The output from running a covered work is covered by this License only if the output, given its content, constitutes a covered work. This License acknowledges your rights of fair use or other equivalent, as provided by copyright law.
You may make, run and propagate covered works that you do not convey, without conditions so long as your license otherwise remains in force. You may convey covered works to others for the sole purpose of having them make modifications exclusively for you, or provide you with facilities for running those works, provided that you comply with the terms of this License in conveying all material for which you do not control copyright. Those thus making or running the covered works for you must do so exclusively on your behalf, under your direction and control, on terms that prohibit them from making any copies of your copyrighted material outside their relationship with you.
Conveying under any other circumstances is permitted solely under the conditions stated below. Sublicensing is not allowed; section 10 makes it unnecessary.
3. Protecting Users' Legal Rights From Anti-Circumvention Law.
No covered work shall be deemed part of an effective technological measure under any applicable law fulfilling obligations under article 11 of the WIPO copyright treaty adopted on 20 December 1996, or similar laws prohibiting or restricting circumvention of such measures.
When you convey a covered work, you waive any legal power to forbid circumvention of technological measures to the extent such circumvention is effected by exercising rights under this License with respect to the covered work, and you disclaim any intention to limit operation or modification of the work as a means of enforcing, against the work's users, your or third parties' legal rights to forbid circumvention of technological measures.
4. Conveying Verbatim Copies.
You may convey verbatim copies of the Program's source code as you receive it, in any medium, provided that you conspicuously and appropriately publish on each copy an appropriate copyright notice; keep intact all notices stating that this License and any non-permissive terms added in accord with section 7 apply to the code; keep intact all notices of the absence of any warranty; and give all recipients a copy of this License along with the Program.
You may charge any price or no price for each copy that you convey, and you may offer support or warranty protection for a fee.
5. Conveying Modified Source Versions.
You may convey a work based on the Program, or the modifications to produce it from the Program, in the form of source code under the terms of section 4, provided that you also meet all of these conditions:
•	a) The work must carry prominent notices stating that you modified it, and giving a relevant date.
•	b) The work must carry prominent notices stating that it is released under this License and any conditions added under section 7. This requirement modifies the requirement in section 4 to “keep intact all notices”.
•	c) You must license the entire work, as a whole, under this License to anyone who comes into possession of a copy. This License will therefore apply, along with any applicable section 7 additional terms, to the whole of the work, and all its parts, regardless of how they are packaged. This License gives no permission to license the work in any other way, but it does not invalidate such permission if you have separately received it.
•	d) If the work has interactive user interfaces, each must display Appropriate Legal Notices; however, if the Program has interactive interfaces that do not display Appropriate Legal Notices, your work need not make them do so.
A compilation of a covered work with other separate and independent works, which are not by their nature extensions of the covered work, and which are not combined with it such as to form a larger program, in or on a volume of a storage or distribution medium, is called an “aggregate” if the compilation and its resulting copyright are not used to limit the access or legal rights of the compilation's users beyond what the individual works permit. Inclusion of a covered work in an aggregate does not cause this License to apply to the other parts of the aggregate.
6. Conveying Non-Source Forms.
You may convey a covered work in object code form under the terms of sections 4 and 5, provided that you also convey the machine-readable Corresponding Source under the terms of this License, in one of these ways:
•	a) Convey the object code in, or embodied in, a physical product (including a physical distribution medium), accompanied by the Corresponding Source fixed on a durable physical medium customarily used for software interchange.
•	b) Convey the object code in, or embodied in, a physical product (including a physical distribution medium), accompanied by a written offer, valid for at least three years and valid for as long as you offer spare parts or customer support for that product model, to give anyone who possesses the object code either (1) a copy of the Corresponding Source for all the software in the product that is covered by this License, on a durable physical medium customarily used for software interchange, for a price no more than your reasonable cost of physically performing this conveying of source, or (2) access to copy the Corresponding Source from a network server at no charge.
•	c) Convey individual copies of the object code with a copy of the written offer to provide the Corresponding Source. This alternative is allowed only occasionally and noncommercially, and only if you received the object code with such an offer, in accord with subsection 6b.
•	d) Convey the object code by offering access from a designated place (gratis or for a charge), and offer equivalent access to the Corresponding Source in the same way through the same place at no further charge. You need not require recipients to copy the Corresponding Source along with the object code. If the place to copy the object code is a network server, the Corresponding Source may be on a different server (operated by you or a third party) that supports equivalent copying facilities, provided you maintain clear directions next to the object code saying where to find the Corresponding Source. Regardless of what server hosts the Corresponding Source, you remain obligated to ensure that it is available for as long as needed to satisfy these requirements.
•	e) Convey the object code using peer-to-peer transmission, provided you inform other peers where the object code and Corresponding Source of the work are being offered to the general public at no charge under subsection 6d.
A separable portion of the object code, whose source code is excluded from the Corresponding Source as a System Library, need not be included in conveying the object code work.
A “User Product” is either (1) a “consumer product”, which means any tangible personal property which is normally used for personal, family, or household purposes, or (2) anything designed or sold for incorporation into a dwelling. In determining whether a product is a consumer product, doubtful cases shall be resolved in favor of coverage. For a particular product received by a particular user, “normally used” refers to a typical or common use of that class of product, regardless of the status of the particular user or of the way in which the particular user actually uses, or expects or is expected to use, the product. A product is a consumer product regardless of whether the product has substantial commercial, industrial or non-consumer uses, unless such uses represent the only significant mode of use of the product.
“Installation Information” for a User Product means any methods, procedures, authorization keys, or other information required to install and execute modified versions of a covered work in that User Product from a modified version of its Corresponding Source. The information must suffice to ensure that the continued functioning of the modified object code is in no case prevented or interfered with solely because modification has been made.
If you convey an object code work under this section in, or with, or specifically for use in, a User Product, and the conveying occurs as part of a transaction in which the right of possession and use of the User Product is transferred to the recipient in perpetuity or for a fixed term (regardless of how the transaction is characterized), the Corresponding Source conveyed under this section must be accompanied by the Installation Information. But this requirement does not apply if neither you nor any third party retains the ability to install modified object code on the User Product (for example, the work has been installed in ROM).
The requirement to provide Installation Information does not include a requirement to continue to provide support service, warranty, or updates for a work that has been modified or installed by the recipient, or for the User Product in which it has been modified or installed. Access to a network may be denied when the modification itself materially and adversely affects the operation of the network or violates the rules and protocols for communication across the network.
Corresponding Source conveyed, and Installation Information provided, in accord with this section must be in a format that is publicly documented (and with an implementation available to the public in source code form), and must require no special password or key for unpacking, reading or copying.
7. Additional Terms.
“Additional permissions” are terms that supplement the terms of this License by making exceptions from one or more of its conditions. Additional permissions that are applicable to the entire Program shall be treated as though they were included in this License, to the extent that they are valid under applicable law. If additional permissions apply only to part of the Program, that part may be used separately under those permissions, but the entire Program remains governed by this License without regard to the additional permissions.
When you convey a copy of a covered work, you may at your option remove any additional permissions from that copy, or from any part of it. (Additional permissions may be written to require their own removal in certain cases when you modify the work.) You may place additional permissions on material, added by you to a covered work, for which you have or can give appropriate copyright permission.
Notwithstanding any other provision of this License, for material you add to a covered work, you may (if authorized by the copyright holders of that material) supplement the terms of this License with terms:
•	a) Disclaiming warranty or limiting liability differently from the terms of sections 15 and 16 of this License; or
•	b) Requiring preservation of specified reasonable legal notices or author attributions in that material or in the Appropriate Legal Notices displayed by works containing it; or
•	c) Prohibiting misrepresentation of the origin of that material, or requiring that modified versions of such material be marked in reasonable ways as different from the original version; or
•	d) Limiting the use for publicity purposes of names of licensors or authors of the material; or
•	e) Declining to grant rights under trademark law for use of some trade names, trademarks, or service marks; or
•	f) Requiring indemnification of licensors and authors of that material by anyone who conveys the material (or modified versions of it) with contractual assumptions of liability to the recipient, for any liability that these contractual assumptions directly impose on those licensors and authors.
All other non-permissive additional terms are considered “further restrictions” within the meaning of section 10. If the Program as you received it, or any part of it, contains a notice stating that it is governed by this License along with a term that is a further restriction, you may remove that term. If a license document contains a further restriction but permits relicensing or conveying under this License, you may add to a covered work material governed by the terms of that license document, provided that the further restriction does not survive such relicensing or conveying.
If you add terms to a covered work in accord with this section, you must place, in the relevant source files, a statement of the additional terms that apply to those files, or a notice indicating where to find the applicable terms.
Additional terms, permissive or non-permissive, may be stated in the form of a separately written license, or stated as exceptions; the above requirements apply either way.
8. Termination.
You may not propagate or modify a covered work except as expressly provided under this License. Any attempt otherwise to propagate or modify it is void, and will automatically terminate your rights under this License (including any patent licenses granted under the third paragraph of section 11).
However, if you cease all violation of this License, then your license from a particular copyright holder is reinstated (a) provisionally, unless and until the copyright holder explicitly and finally terminates your license, and (b) permanently, if the copyright holder fails to notify you of the violation by some reasonable means prior to 60 days after the cessation.
Moreover, your license from a particular copyright holder is reinstated permanently if the copyright holder notifies you of the violation by some reasonable means, this is the first time you have received notice of violation of this License (for any work) from that copyright holder, and you cure the violation prior to 30 days after your receipt of the notice.
Termination of your rights under this section does not terminate the licenses of parties who have received copies or rights from you under this License. If your rights have been terminated and not permanently reinstated, you do not qualify to receive new licenses for the same material under section 10.
9. Acceptance Not Required for Having Copies.
You are not required to accept this License in order to receive or run a copy of the Program. Ancillary propagation of a covered work occurring solely as a consequence of using peer-to-peer transmission to receive a copy likewise does not require acceptance. However, nothing other than this License grants you permission to propagate or modify any covered work. These actions infringe copyright if you do not accept this License. Therefore, by modifying or propagating a covered work, you indicate your acceptance of this License to do so.
10. Automatic Licensing of Downstream Recipients.
Each time you convey a covered work, the recipient automatically receives a license from the original licensors, to run, modify and propagate that work, subject to this License. You are not responsible for enforcing compliance by third parties with this License.
An “entity transaction” is a transaction transferring control of an organization, or substantially all assets of one, or subdividing an organization, or merging organizations. If propagation of a covered work results from an entity transaction, each party to that transaction who receives a copy of the work also receives whatever licenses to the work the party's predecessor in interest had or could give under the previous paragraph, plus a right to possession of the Corresponding Source of the work from the predecessor in interest, if the predecessor has it or can get it with reasonable efforts.
You may not impose any further restrictions on the exercise of the rights granted or affirmed under this License. For example, you may not impose a license fee, royalty, or other charge for exercise of rights granted under this License, and you may not initiate litigation (including a cross-claim or counterclaim in a lawsuit) alleging that any patent claim is infringed by making, using, selling, offering for sale, or importing the Program or any portion of it.
11. Patents.
A “contributor” is a copyright holder who authorizes use under this License of the Program or a work on which the Program is based. The work thus licensed is called the contributor's “contributor version”.
A contributor's “essential patent claims” are all patent claims owned or controlled by the contributor, whether already acquired or hereafter acquired, that would be infringed by some manner, permitted by this License, of making, using, or selling its contributor version, but do not include claims that would be infringed only as a consequence of further modification of the contributor version. For purposes of this definition, “control” includes the right to grant patent sublicenses in a manner consistent with the requirements of this License.
Each contributor grants you a non-exclusive, worldwide, royalty-free patent license under the contributor's essential patent claims, to make, use, sell, offer for sale, import and otherwise run, modify and propagate the contents of its contributor version.
In the following three paragraphs, a “patent license” is any express agreement or commitment, however denominated, not to enforce a patent (such as an express permission to practice a patent or covenant not to sue for patent infringement). To “grant” such a patent license to a party means to make such an agreement or commitment not to enforce a patent against the party.
If you convey a covered work, knowingly relying on a patent license, and the Corresponding Source of the work is not available for anyone to copy, free of charge and under the terms of this License, through a publicly available network server or other readily accessible means, then you must either (1) cause the Corresponding Source to be so available, or (2) arrange to deprive yourself of the benefit of the patent license for this particular work, or (3) arrange, in a manner consistent with the requirements of this License, to extend the patent license to downstream recipients. “Knowingly relying” means you have actual knowledge that, but for the patent license, your conveying the covered work in a country, or your recipient's use of the covered work in a country, would infringe one or more identifiable patents in that country that you have reason to believe are valid.
If, pursuant to or in connection with a single transaction or arrangement, you convey, or propagate by procuring conveyance of, a covered work, and grant a patent license to some of the parties receiving the covered work authorizing them to use, propagate, modify or convey a specific copy of the covered work, then the patent license you grant is automatically extended to all recipients of the covered work and works based on it.
A patent license is “discriminatory” if it does not include within the scope of its coverage, prohibits the exercise of, or is conditioned on the non-exercise of one or more of the rights that are specifically granted under this License. You may not convey a covered work if you are a party to an arrangement with a third party that is in the business of distributing software, under which you make payment to the third party based on the extent of your activity of conveying the work, and under which the third party grants, to any of the parties who would receive the covered work from you, a discriminatory patent license (a) in connection with copies of the covered work conveyed by you (or copies made from those copies), or (b) primarily for and in connection with specific products or compilations that contain the covered work, unless you entered into that arrangement, or that patent license was granted, prior to 28 March 2007.
Nothing in this License shall be construed as excluding or limiting any implied license or other defenses to infringement that may otherwise be available to you under applicable patent law.
12. No Surrender of Others' Freedom.
If conditions are imposed on you (whether by court order, agreement or otherwise) that contradict the conditions of this License, they do not excuse you from the conditions of this License. If you cannot convey a covered work so as to satisfy simultaneously your obligations under this License and any other pertinent obligations, then as a consequence you may not convey it at all. For example, if you agree to terms that obligate you to collect a royalty for further conveying from those to whom you convey the Program, the only way you could satisfy both those terms and this License would be to refrain entirely from conveying the Program.
13. Use with the GNU Affero General Public License.
Notwithstanding any other provision of this License, you have permission to link or combine any covered work with a work licensed under version 3 of the GNU Affero General Public License into a single combined work, and to convey the resulting work. The terms of this License will continue to apply to the part which is the covered work, but the special requirements of the GNU Affero General Public License, section 13, concerning interaction through a network will apply to the combination as such.
14. Revised Versions of this License.
The Free Software Foundation may publish revised and/or new versions of the GNU General Public License from time to time. Such new versions will be similar in spirit to the present version, but may differ in detail to address new problems or concerns.
Each version is given a distinguishing version number. If the Program specifies that a certain numbered version of the GNU General Public License “or any later version” applies to it, you have the option of following the terms and conditions either of that numbered version or of any later version published by the Free Software Foundation. If the Program does not specify a version number of the GNU General Public License, you may choose any version ever published by the Free Software Foundation.
If the Program specifies that a proxy can decide which future versions of the GNU General Public License can be used, that proxy's public statement of acceptance of a version permanently authorizes you to choose that version for the Program.
Later license versions may give you additional or different permissions. However, no additional obligations are imposed on any author or copyright holder as a result of your choosing to follow a later version.
15. Disclaimer of Warranty.
THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
16. Limitation of Liability.
IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MODIFIES AND/OR CONVEYS THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
17. Interpretation of Sections 15 and 16.
If the disclaimer of warranty and limitation of liability provided above cannot be given local legal effect according to their terms, reviewing courts shall apply local law that most closely approximates an absolute waiver of all civil liability in connection with the Program, unless a warranty or assumption of liability accompanies a copy of the Program in return for a fee.
END OF TERMS AND CONDITIONS
How to Apply These Terms to Your New Programs
If you develop a new program, and you want it to be of the greatest possible use to the public, the best way to achieve this is to make it free software which everyone can redistribute and change under these terms.
To do so, attach the following notices to the program. It is safest to attach them to the start of each source file to most effectively state the exclusion of warranty; and each file should have at least the “copyright” line and a pointer to where the full notice is found.
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
Also add information on how to contact you by electronic and paper mail.
If the program does terminal interaction, make it output a short notice like this when it starts in an interactive mode:
    <program>  Copyright (C) <year>  <name of author>
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
The hypothetical commands `show w' and `show c' should show the appropriate parts of the General Public License. Of course, your program's commands might be different; for a GUI interface, you would use an “about box”.
You should also get your employer (if you work as a programmer) or school, if any, to sign a “copyright disclaimer” for the program, if necessary. For more information on this, and how to apply and follow the GNU GPL, see <http://www.gnu.org/licenses/>.
The GNU General Public License does not permit incorporating your program into proprietary programs. If your program is a subroutine library, you may consider it more useful to permit linking proprietary applications with the library. If this is what you want to do, use the GNU Lesser General Public License instead of this License. But first, please read <http://www.gnu.org/philosophy/why-not-lgpl.html>.

*/
