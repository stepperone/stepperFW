#include "screens.h"
#include "main.h"
#include "EPD_2in66.h"
#include "DEV_Config.h"
#include "ImageData.h"
#include "GUI_Paint.h"
#include "buttons.h"

#define MAX_PARTIAL_REFRESH 10
#define LABEL_TIMEOUT 5000

#define LOGO_HEIGHT 230

UBYTE *dispCanvas;

void checkFlashSize() // don't actually write this to the display. This is just so all resources get compiled in to check flash capacity while testing
{
    prepCanvas();
    drawLogo(LOGO_HEIGHT);
    Draw_CenteredText(25, "203", &Font_counter, WHITE, BLACK);
    Paint_DrawBitMap_Paste(steps, 20, 90, 38, 30, BLACK);
    Paint_DrawString_EN(60, 90, "steps", &Font_logo1, WHITE, BLACK);

    Paint_DrawBitMap_Paste(clock, 25, 175, 21, 21, BLACK);
    Paint_DrawString_EN(50, 165, "Hour", &Font_logo2, WHITE, BLACK);
    Paint_DrawString_EN(50, 185, "124", &Font_logo2, WHITE, BLACK);

    Paint_DrawBitMap_Paste(achievement, 25, 230, 21, 21, BLACK);
    Paint_DrawString_EN(50, 220, "Record", &Font_logo2, WHITE, BLACK);
    Paint_DrawString_EN(50, LOGO_HEIGHT, "12532", &Font_logo2, WHITE, BLACK);
    prepCanvas();
    Paint_DrawBitMap_Paste(cone, 42, 10, 68, 65, BLACK);
    Draw_CenteredText(75, "error", &Font_title, WHITE, BLACK);
    Draw_CenteredText(100, "occured at", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(120, "self-test", &Font_logo2, WHITE, BLACK);
    prepCanvas();
    Paint_DrawBitMap_Paste(table, 42, 10, 68, 70, BLACK);
    Draw_CenteredText(80, "calibration", &Font_title, WHITE, BLACK);
    Draw_CenteredText(105, "steady the", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(125, "device", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(170, "one moment", &Font_logo2, WHITE, BLACK);
    prepCanvas();
    Paint_DrawBitMap_Paste(shredder, 43, 10, 65, 72, BLACK);
    Draw_CenteredText(80, "reset steps", &Font_title, WHITE, BLACK);
    Draw_CenteredText(105, "select stat", &Font_logo2, WHITE, BLACK);
}

void powerOff()
{
    prepCanvas();
    drawLogo(LOGO_HEIGHT);
}

void homescreen()
{
    isButtonPressed(0); // we call this function to check for updated times and if to display the labels
    Paint_SelectImage(dispCanvas);
    Paint_SetRotate(0);
    Paint_Clear(WHITE);
    char sbuffer[12]; // good up to 10-digit int + sign + '\0'
    snprintf(sbuffer, sizeof(sbuffer), "%d", stepper.steps);
    char *sString = sbuffer;
    // clear window
    Draw_CenteredText(25, sString, &Font_counter, WHITE, BLACK);

    Paint_DrawBitMap_Paste(steps, 20, 90, 38, 30, BLACK);
    Paint_DrawString_EN(60, 90, "steps", &Font_logo1, WHITE, BLACK);

    char hbuffer[12]; // good up to 10-digit int + sign + '\0'
    snprintf(hbuffer, sizeof(hbuffer), "%d", stepper.hour);
    char *hString = hbuffer;

    Paint_DrawBitMap_Paste(clock, 25, 175, 21, 21, BLACK);
    Paint_DrawString_EN(50, 165, "Hour", &Font_logo2, WHITE, BLACK);
    Paint_DrawString_EN(50, 185, hString, &Font_logo2, WHITE, BLACK);

    char rbuffer[12]; // good up to 10-digit int + sign + '\0'
    if (stepper.record < stepper.steps) {
    	stepper.record = stepper.steps;
    }
    snprintf(rbuffer, sizeof(rbuffer), "%d", stepper.record);
    char *rString = rbuffer;

    Paint_DrawBitMap_Paste(achievement, 25, 230, 21, 21, BLACK);
    Paint_DrawString_EN(50, 216, "Record", &Font_logo2, WHITE, BLACK);
    Paint_DrawString_EN(50, 236, rString, &Font_logo2, WHITE, BLACK);

    if (stepper.showButtons)
    {
        Draw_CenteredText(25, getBatteryState(), &Font_counter, WHITE, BLACK);
        Paint_SetRotate(270);
        Paint_DrawBitMap_Paste(triangle, 140, 130, 8, 5, BLACK);
        Paint_DrawString_EN(100, 135, "Calibration", &Font_label, WHITE, BLACK);
        Paint_DrawBitMap_Paste(triangle, 31, 130, 8, 5, BLACK);
        Paint_DrawString_EN(16, 135, "Reset", &Font_label, WHITE, BLACK);
    }
    Paint_SetRotate(0);
}

void error()
{
    prepCanvas();
    Paint_DrawBitMap_Paste(cone, 42, 10, 68, 65, BLACK);
    Draw_CenteredText(75, "error", &Font_title, WHITE, BLACK);
    Draw_CenteredText(100, "occured at", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(120, "self-test", &Font_logo2, WHITE, BLACK);
    drawLogo(LOGO_HEIGHT);
}

void calibration()
{
    prepCanvas();
    Paint_DrawBitMap_Paste(table, 42, 10, 68, 70, BLACK);
    Draw_CenteredText(80, "calibration", &Font_title, WHITE, BLACK);
    Draw_CenteredText(105, "steady the", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(125, "device", &Font_logo2, WHITE, BLACK);
    Draw_CenteredText(170, "one moment", &Font_logo2, WHITE, BLACK);
    drawLogo(LOGO_HEIGHT);
}

void buttonlessCalibration(int rotation) {
	prepCanvas();

	if (rotation >= 0) {
		Paint_SetRotate(rotation);
		Draw_CenteredText(80, "UP", &Font_title, WHITE, BLACK);
	} else if (rotation == -1) { // face up for z
		Paint_SetRotate(0);
		Draw_CenteredText(80, "Screen UP", &Font_title, WHITE, BLACK);
	} else if (rotation == -2) { // face down for z
		Paint_SetRotate(0);
		Draw_CenteredText(80, "Screen DOWN", &Font_title, WHITE, BLACK);
	} else {
		Paint_SetRotate(0);
		Draw_CenteredText(80, "Error", &Font_title, WHITE, BLACK);
	}

	Paint_SetRotate(0); // just to be safe, this should work fine if removed;
}

void reset()
{
    prepCanvas();
    Paint_DrawBitMap_Paste(shredder, 43, 10, 65, 72, BLACK);
    Draw_CenteredText(80, "reset steps", &Font_title, WHITE, BLACK);
    Draw_CenteredText(105, "select stat", &Font_logo2, WHITE, BLACK);
    Paint_SetRotate(270);
    Paint_DrawBitMap_Paste(triangle, 31, 135, 8, 5, BLACK);
    Paint_DrawString_EN(16, 140, "Back", &Font_label, WHITE, BLACK);
    Paint_SetRotate(0);
    drawLogo(LOGO_HEIGHT);
}

void test()
{
    prepCanvas();
}

void demoImg()
{
    prepCanvas();
    Paint_DrawBitMap(stepperHome);
}

void drawLogo(int height)
{
    Paint_DrawString_EN(7, height, "stepper", &Font_logo1, WHITE, BLACK);
    Paint_DrawString_EN(107, height - 4, "one", &Font_logo2, WHITE, BLACK);
}

/////// helper functions below

void prepCanvas()
{
    Paint_SelectImage(dispCanvas);
    Paint_SetRotate(0);
    Paint_Clear(WHITE);
}

void Draw_CenteredText(int y, const char *text, sFONT *font, uint16_t bg_color, uint16_t fg_color)
{
    if (!text || !font)
        return;

    int text_width = strlen(text) * font->Width;
    int x = (Paint.Width - text_width) / 2;

    Paint_DrawString_EN(x, y, text, font, bg_color, fg_color);
}

void copyState(struct stepperStruct *lastState)
{
    lastState->steps = stepper.steps;
    lastState->hour = stepper.hour;
    lastState->record = stepper.record;
    lastState->pace = stepper.pace;
    lastState->screen = stepper.screen;
    lastState->showButtons = stepper.showButtons;
}

void manageDisp()
{
	uint32_t now = HAL_GetTick();
    if ((stepper.pace == IDLE) && ((now - stepper.lastStepTime) > 1000))
    {
        static int initLastState = 0;
        static struct stepperStruct lastState;
        static int partialRefreshCount = 0;

        if (HAL_GetTick() - stepper.lastButtonPress < LABEL_TIMEOUT && stepper.lastButtonPress != 9999999)
        {
            stepper.showButtons = 1;
        }
        else
        {
            stepper.showButtons = 0;
        }

        if (!initLastState)
        {
            copyState(&lastState);
            // we update the flag variable in the refresh display mechanism, so it refreshes upon startup
        }

        int refreshNeeded = 0;

        if ((stepper.screen == HOMESCREEN && stepper.showButtons != lastState.showButtons) || stepper.screen != lastState.screen ||
            stepper.steps != lastState.steps ||
            stepper.hour != lastState.hour ||
            stepper.record != lastState.record || !initLastState)
        {

            switch (stepper.screen)
            {
            case PWR_OFF:
                powerOff();
                break;
            case SYS_ERROR:
                error();
                break;
            case CALIBRATION:
                calibration();
                break;
            case CALIBRATE_Y_UP:
            	buttonlessCalibration(0);
            	break;
            case CALIBRATE_Y_DOWN:
            	buttonlessCalibration(180);
            	break;
            case CALIBRATE_Z_UP:
            	buttonlessCalibration(270);
            	break;
            case CALIBRATE_Z_DOWN:
            	buttonlessCalibration(90);
            	break;
            case CALIBRATE_X_UP:
            	buttonlessCalibration(-2);
            	break;
            case CALIBRATE_X_DOWN:
            	buttonlessCalibration(-1);
            	break;
            case SYS_RESET:
                reset();
                break;
            default:
                homescreen();
                break;
            }

            // printf("==SOMETHING CHANGED==\r\n");
            // printf("NEW: %d %d %d %d, OLD:  %d %d %d %d\r\n", stepper.steps, stepper.hour, stepper.record, stepper.showButtons, lastState.steps, lastState.hour, lastState.record, lastState.showButtons);
            initLastState = 1;
            refreshNeeded = 1;
        }

        // printf("fullRefreshNeeded: %d, PartialCount: %d\r\n", fullRefreshNeeded, partialRefreshCount);

        if (refreshNeeded && stepper.pace == IDLE || !initLastState) // only update the display if info changes and the device is not in motion
        {
            if (partialRefreshCount >= MAX_PARTIAL_REFRESH || stepper.screen == HOMESCREEN && lastState.screen != HOMESCREEN)
            {
                EPD_2IN66_Init();
                printf("////FULL REFRESH////\r\n");
                EPD_2IN66_Display(dispCanvas);
                EPD_2IN66_Init_Partial();
                partialRefreshCount = 0;
                updateRecordEE();
            }
            else
            {
                printf("////PARTIAL REFRESH////\r\n");
                EPD_2IN66_Display(dispCanvas);
                partialRefreshCount++;
            }
            copyState(&lastState);
        }
    }
}

void writeDispShutdown()
{
    EPD_2IN66_Display(dispCanvas);
    DEV_Delay_ms(2000);
    EPD_2IN66_Sleep();
    DEV_Delay_ms(1000);
    DEV_Module_Exit();
}

void setupDisp()
{
    // printf("Initilising dev module\r\n");
    if (DEV_Module_Init() != 0)
    {
        return -1;
    }

//    printf("e-Paper Init and Clear...\r\n");
    EPD_2IN66_Init();
    EPD_2IN66_Clear();
    DEV_Delay_ms(500);

    UWORD Imagesize = ((EPD_2IN66_WIDTH % 8 == 0) ? (EPD_2IN66_WIDTH / 8) : (EPD_2IN66_WIDTH / 8 + 1)) * EPD_2IN66_HEIGHT;
    if ((dispCanvas = (UBYTE *)malloc(Imagesize)) == NULL)
    {
        printf("Failed to apply for image memory...\r\n");
        return -1;
    }
    EPD_2IN66_Init_Partial();
    // printf("Created new paint image\r\n");
    Paint_NewImage(dispCanvas, EPD_2IN66_WIDTH, EPD_2IN66_HEIGHT, 0, WHITE); // zero reps orientation of disp
    printf("Display setup done!...\r\n");
}

// make buttons only respond if not busy
// ensure the lastState update only occurs if the display is not busy??
