#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "dma.h"
#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "timer_sdk.h"
#include "mmio.h"

#include "safa.h"
#include "main.h"

/*
 * Se mantienen las estructuras DMA estáticas para que todos los campos no
 * utilizados queden inicializados a cero. Esto evita valores indeterminados
 * en configuraciones opcionales del DMA.
 */
static dma_trans_t trans;
static dma_target_t tgt_src;
static dma_target_t tgt_dst;

static safa_t safa;

static int compare_result(void)
{
    int errors = 0;

    for (uint32_t i = 0; i < OUTPUT_SIZE_BYTES; i++) {
        if (image_gray_dst[i] != image_gray[i]) {
            if (errors < 20) {
                printf(
                    "Error pixel %lu: esperado = 0x%02X, "
                    "obtenido = 0x%02X, diferencia = %d\n\r",
                    (unsigned long)i,
                    image_gray[i],
                    image_gray_dst[i],
                    abs((int)image_gray[i] - (int)image_gray_dst[i])
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
        error_total += abs(
            (int)image_gray[i] - (int)image_gray_dst[i]
        );
    }

    return (int)(error_total / OUTPUT_SIZE_BYTES);
}

static const char *safa_result_to_string(safa_result_t result)
{
    switch (result) {
        case SAFA_RESULT_OK:
            return "OK";
        case SAFA_RESULT_BAD_ARGUMENT:
            return "argumento no valido";
        case SAFA_RESULT_BUSY:
            return "periferico ocupado";
        case SAFA_RESULT_TIMEOUT:
            return "timeout";
        case SAFA_RESULT_ABORTED:
            return "transaccion abortada";
        case SAFA_RESULT_HARDWARE_ERROR:
            return "error hardware";
        case SAFA_RESULT_NOT_STARTED:
            return "transaccion no iniciada";
        case SAFA_RESULT_VERSION_MISMATCH:
            return "version RTL/driver incompatible";
        default:
            return "resultado desconocido";
    }
}

static void print_safa_errors(uint32_t errors)
{
    if (errors == 0u) {
        printf("SAFA ERROR_STATUS = 0x00000000\n\r");
        return;
    }

    printf("SAFA ERROR_STATUS = 0x%08lX\n\r", (unsigned long)errors);

    if ((errors & SAFA_ERROR_START_WHILE_BUSY_MASK) != 0u) {
        printf("  - START solicitado mientras SAFA estaba ocupado\n\r");
    }

    if ((errors & SAFA_ERROR_DMA_PUSH_BLOCKED_MASK) != 0u) {
        printf("  - El DMA intento escribir con la FIFO de entrada bloqueada\n\r");
    }

    if ((errors & SAFA_ERROR_DMA_POP_EMPTY_MASK) != 0u) {
        printf("  - El DMA intento leer con la FIFO de salida vacia\n\r");
    }

    if ((errors & SAFA_ERROR_HLS_READ_EMPTY_MASK) != 0u) {
        printf("  - El HLS intento leer con la FIFO de entrada vacia\n\r");
    }

    if ((errors & SAFA_ERROR_HLS_WRITE_FULL_MASK) != 0u) {
        printf("  - El HLS intento escribir con la FIFO de salida llena\n\r");
    }

    if ((errors & SAFA_ERROR_UNEXPECTED_AP_DONE_MASK) != 0u) {
        printf("  - Se recibio ap_done en un estado inesperado\n\r");
    }

    if ((errors & SAFA_ERROR_INPUT_COUNT_MISMATCH_MASK) != 0u) {
        printf("  - El numero de entradas consumidas no coincide\n\r");
    }

    if ((errors & SAFA_ERROR_OUTPUT_COUNT_MISMATCH_MASK) != 0u) {
        printf("  - El numero de salidas generadas no coincide\n\r");
    }

    if ((errors & SAFA_ERROR_INPUT_FIFO_NOT_EMPTY_MASK) != 0u) {
        printf("  - La FIFO de entrada no estaba vacia al finalizar\n\r");
    }
}

static int configure_safa(void)
{
    safa_result_t result;

    result = safa_init(
        &safa,
        mmio_region_from_addr((uintptr_t)SAFA_PERIPH_START_ADDRESS)
    );

    if (result != SAFA_RESULT_OK) {
        printf(
            "Error inicializando SAFA: %s (%d)\n\r",
            safa_result_to_string(result),
            (int)result
        );
        return -1;
    }

    /*
     * AUTO_START conserva el comportamiento del wrapper antiguo:
     * el primer push aceptado desde el DMA genera el arranque del HLS.
     *
     * Las interrupciones de SAFA permanecen deshabilitadas porque este
     * programa utiliza polling, igual que la versión original.
     */
    const safa_config_t config = {
        .input_words = INPUT_SIZE_WORDS,
        .output_words = OUTPUT_SIZE_WORDS,
        .auto_start = true,
        .irq_enable_mask = 0u,
    };

    result = safa_configure(&safa, &config);

    if (result != SAFA_RESULT_OK) {
        printf(
            "Error configurando SAFA: %s (%d)\n\r",
            safa_result_to_string(result),
            (int)result
        );
        return -1;
    }

    printf(
        "SAFA inicializado. Version RTL = 0x%08lX\n\r",
        (unsigned long)safa_get_version(&safa)
    );

    return 0;
}

static int configure_dma(void)
{
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
     * Como tgt_src.type y tgt_dst.type son DMA_DATA_TYPE_WORD,
     * size_d1_du indica palabras de 32 bits.
     */
    trans.size_d1_du = INPUT_SIZE_WORDS;

    /* Se conserva la configuración de finalización original. */
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
    return 0;
}

static int wait_for_accelerator(void)
{
    /*
     * Se conserva la espera activa del código original, pero se consulta
     * también el estado de SAFA para detectar un fallo antes de que el DMA
     * termine.
     */
    while (!dma_is_ready(DMA_CHANNEL_ACCELERATOR)) {
        safa_status_t status;
        safa_result_t result = safa_get_status(&safa, &status);

        if (result != SAFA_RESULT_OK) {
            printf(
                "Error leyendo el estado de SAFA: %s (%d)\n\r",
                safa_result_to_string(result),
                (int)result
            );
            return -1;
        }

        if (status.error) {
            printf("SAFA ha detectado un error durante la ejecucion\n\r");
            print_safa_errors(safa_get_errors(&safa));
            (void)safa_abort(&safa);
            return -1;
        }

        if (status.aborted) {
            printf("La transaccion SAFA ha sido abortada\n\r");
            return -1;
        }
    }

    /*
     * En modo HW FIFO el DMA debería finalizar cuando hw_fifo_done_o se
     * activa. Esta comprobación adicional confirma el estado sticky DONE
     * expuesto por la interfaz de registros.
     */
    safa_result_t result = safa_wait_done(&safa, SAFA_WAIT_FOREVER);

    if (result != SAFA_RESULT_OK) {
        printf(
            "SAFA no termino correctamente: %s (%d)\n\r",
            safa_result_to_string(result),
            (int)result
        );
        print_safa_errors(safa_get_errors(&safa));
        return -1;
    }

    return 0;
}

static void print_safa_counters(void)
{
    safa_counters_t counters;

    if (safa_get_counters(&safa, &counters) != SAFA_RESULT_OK) {
        printf("No se pudieron leer los contadores de SAFA\n\r");
        return;
    }

    printf("Contadores SAFA:\n\r");
    printf(
        "  Entrada aceptada  = %lu words\n\r",
        (unsigned long)counters.input_accepted
    );
    printf(
        "  Entrada consumida = %lu words\n\r",
        (unsigned long)counters.input_consumed
    );
    printf(
        "  Salida generada   = %lu words\n\r",
        (unsigned long)counters.output_generated
    );
    printf(
        "  Salida extraida   = %lu words\n\r",
        (unsigned long)counters.output_popped
    );
}

int main(void)
{
    uint32_t total_cycles = 0;

    memset(image_gray_dst, 0, OUTPUT_SIZE_BYTES);

    timer_cycles_init();

    dma_init(NULL);

    printf("Init DMA\n\r");
    printf(
        "Input size  = %d bytes = %d words\n\r",
        INPUT_SIZE_BYTES,
        INPUT_SIZE_WORDS
    );
    printf(
        "Output size = %d bytes = %d words\n\r",
        OUTPUT_SIZE_BYTES,
        OUTPUT_SIZE_WORDS
    );

    if (configure_safa() != 0) {
        return -1;
    }

    if (configure_dma() != 0) {
        return -1;
    }

    printf(
        "Lanzando DMA en canal %d...\n\r",
        DMA_CHANNEL_ACCELERATOR
    );

    timer_start();

    /*
     * No se llama a safa_start(): AUTO_START está habilitado y el primer
     * push del DMA inicia automáticamente el acelerador.
     */
    dma_launch(&trans);

    if (wait_for_accelerator() != 0) {
        total_cycles = timer_stop();
        printf(
            "Ejecucion interrumpida tras %lu ciclos\n\r",
            (unsigned long)total_cycles
        );
        return -1;
    }

    total_cycles = timer_stop();

    printf("DMA y SAFA terminados\n\r");
    printf(
        "Ciclos de reloj: %lu cc\n\r",
        (unsigned long)total_cycles
    );
    printf(
        "Tiempo: %lu us\n\r",
        (unsigned long)get_time_from_cycles(total_cycles)
    );

    print_safa_counters();

    printf("Primeros valores de image_gray_dst:\n\r");

    for (int i = 0; i < 32; i++) {
        printf("dst[%d] = 0x%02X\n\r", i, image_gray_dst[i]);
    }

    printf("Comparando contra image_gray...\n\r");

    int errors = compare_result();

    /* Limpia el estado DONE para dejar el periférico preparado. */
    (void)safa_clear_done(&safa);

    if (errors == 0) {
        printf("TEST OK: image_gray_dst coincide con image_gray\n\r");
        return 0;
    }

    printf("TEST FAIL: diferencias encontradas = %d\n\r", errors);

    int m_error = mean_error();
    printf("Error medio en la imagen = %d\n\r", m_error);

    return 1;
}