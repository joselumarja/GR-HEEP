#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "dma.h"
#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "timer_sdk.h"

#include "main.h" 

static int compare_result(void)
{
    int errors = 0;

    for (uint32_t i = 0; i < OUTPUT_SIZE_BYTES; i++) {
        if (image_gray_dst[i] != image_gray[i]) {
            if (errors < 20) {
                printf(
                    "Error pixel %lu: esperado = 0x%02X, obtenido = 0x%02X, diferencia = %d\n\r",
                    (unsigned long)i,
                    image_gray[i],
                    image_gray_dst[i],
                    abs(image_gray[i]-image_gray_dst[i])
                );
            }

            errors++;
        }
    }

    return errors;
}

static int mean_error(void)
{
    long long error_total = 0;

    for (uint32_t i = 0; i < OUTPUT_SIZE_BYTES; i++) {
        if (image_gray_dst[i] != image_gray[i]) {

            error_total += abs(image_gray[i]-image_gray_dst[i]);

        }
    }

    return error_total/OUTPUT_SIZE_BYTES;
}

int main(void)
{
    dma_trans_t trans;
    dma_target_t tgt_src;
    dma_target_t tgt_dst;

    uint32_t total_cycles = 0;

    memset(image_gray_dst, 0, OUTPUT_SIZE_BYTES);

    timer_cycles_init();

    dma_init(NULL);

    printf("Init DMA\n\r");
    printf("Input size  = %d bytes = %d words\n\r", INPUT_SIZE_BYTES, INPUT_SIZE_WORDS);
    printf("Output size = %d bytes = %d words\n\r", OUTPUT_SIZE_BYTES, OUTPUT_SIZE_WORDS);

    /*
     * Fuente: imagen BGR/RGB en memoria.
     * Transferencia en palabras de 32 bits.
     */
    tgt_src.ptr = (uint8_t *)image_bgr;
    tgt_src.inc_d1_du = 1;
    tgt_src.trig = DMA_TRIG_MEMORY;
    tgt_src.type = DMA_DATA_TYPE_WORD;

    /*
     * Destino: buffer de salida.
     * También en palabras de 32 bits.
     */
    tgt_dst.ptr = (uint8_t *)image_gray_dst;
    tgt_dst.inc_d1_du = 1;
    tgt_dst.trig = DMA_TRIG_MEMORY;
    tgt_dst.type = DMA_DATA_TYPE_WORD;

    trans.src = &tgt_src;
    trans.dst = &tgt_dst;

    trans.mode = DMA_TRANS_MODE_SINGLE;
    trans.hw_fifo_en = 1;
    trans.channel = DMA_CHANNEL_ACCELERATOR;
    trans.dim = DMA_DIM_CONF_1D;

    /*
     * IMPORTANTE:
     *
     * Como tgt_src.type y tgt_dst.type son DMA_DATA_TYPE_WORD,
     * size_d1_du indica palabras de 32 bits.
     *
     * Para la entrada:
     *   6912 bytes / 4 = 1728 words
     */
    trans.size_d1_du = INPUT_SIZE_WORDS;

    trans.end = DMA_TRANS_END_INTR;

    if (dma_validate_transaction(
            &trans,
            DMA_ENABLE_REALIGN,
            DMA_PERFORM_CHECKS_INTEGRITY
        ) != DMA_CONFIG_OK) {

        printf("Error validando DMA\n\r");
        return -1;
    }

    dma_load_transaction(&trans);

    printf("Lanzando DMA en canal %d...\n\r", DMA_CHANNEL_ACCELERATOR);

    timer_start();

    dma_launch(&trans);

    while (!dma_is_ready(DMA_CHANNEL_ACCELERATOR)) {
        /*
         * Espera activa.
         */
    }

    total_cycles = timer_stop();

    printf("DMA terminado\n\r");
    printf("Ciclos de reloj: %lu cc\n\r", (unsigned long)total_cycles);
    printf("Tiempo: %lu us\n\r", (unsigned long)get_time_from_cycles(total_cycles));

    printf("Primeros valores de image_gray_dst:\n\r");

    for (int i = 0; i < 32; i++) {
        printf("dst[%d] = 0x%02X\n\r", i, image_gray_dst[i]);
    }

    printf("Comparando contra image_gray...\n\r");

    int errors = compare_result();

    if (errors == 0) {
        printf("TEST OK: image_gray_dst coincide con image_gray\n\r");
        return 0;
    } else {
        printf("TEST FAIL: diferencias encontradas = %d\n\r", errors);

        int m_error = mean_error();
        printf("Error medio en la imagen = %d\n\r", m_error);

        return 1;
    }

}