/* ML-KEM encapsulate/decapsulate round-trip test via pkcs11-provider.
 * Works for any parameter set (512/768/1024) — the object itself
 * determines which one is used.
 *
 * Usage: mlkem_roundtrip <object-label>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <openssl/evp.h>
#include <openssl/store.h>
#include <openssl/err.h>

static void print_openssl_errors(void)
{
	unsigned long e;

	while ((e = ERR_get_error()) != 0) {
		char buf[256];

		ERR_error_string_n(e, buf, sizeof(buf));
		fprintf(stderr, "  openssl error: %s\n", buf);
	}
}

static EVP_PKEY *load_pkey(const char *uri)
{
	OSSL_STORE_CTX *ctx = NULL;
	OSSL_STORE_INFO *info = NULL;
	EVP_PKEY *pkey = NULL;

	ctx = OSSL_STORE_open_ex(uri, NULL, "provider=pkcs11", NULL, NULL,
				 NULL, NULL, NULL);
	if (!ctx) {
		fprintf(stderr, "OSSL_STORE_open_ex(%s) failed\n", uri);
		print_openssl_errors();
		return NULL;
	}

	while (!OSSL_STORE_eof(ctx)) {
		int type;

		info = OSSL_STORE_load(ctx);
		if (!info)
			continue;

		type = OSSL_STORE_INFO_get_type(info);
		if (type == OSSL_STORE_INFO_PKEY) {
			pkey = OSSL_STORE_INFO_get1_PKEY(info);
			OSSL_STORE_INFO_free(info);
			break;
		}
		if (type == OSSL_STORE_INFO_PUBKEY) {
			pkey = OSSL_STORE_INFO_get1_PUBKEY(info);
			OSSL_STORE_INFO_free(info);
			break;
		}
		OSSL_STORE_INFO_free(info);
	}

	OSSL_STORE_close(ctx);

	if (!pkey) {
		fprintf(stderr, "No EVP_PKEY found at %s\n", uri);
		print_openssl_errors();
	}

	return pkey;
}

static int do_encapsulate(EVP_PKEY *pub, unsigned char **ct, size_t *ct_len,
			  unsigned char **secret, size_t *secret_len)
{
	EVP_PKEY_CTX *pctx = NULL;
	int ret = -1;

	pctx = EVP_PKEY_CTX_new_from_pkey(NULL, pub, "provider=pkcs11");
	if (!pctx) {
		fprintf(stderr, "EVP_PKEY_CTX_new_from_pkey (encaps) failed\n");
		print_openssl_errors();
		return -1;
	}

	if (EVP_PKEY_encapsulate_init(pctx, NULL) <= 0) {
		fprintf(stderr, "EVP_PKEY_encapsulate_init failed\n");
		print_openssl_errors();
		goto out;
	}

	*ct_len = 0;
	*secret_len = 0;
	if (EVP_PKEY_encapsulate(pctx, NULL, ct_len, NULL, secret_len) <= 0) {
		fprintf(stderr, "EVP_PKEY_encapsulate (size query) failed\n");
		print_openssl_errors();
		goto out;
	}

	*ct = malloc(*ct_len);
	*secret = malloc(*secret_len);
	if (!*ct || !*secret) {
		fprintf(stderr, "malloc failed\n");
		goto out;
	}

	if (EVP_PKEY_encapsulate(pctx, *ct, ct_len, *secret, secret_len) <= 0) {
		fprintf(stderr, "EVP_PKEY_encapsulate failed\n");
		print_openssl_errors();
		goto out;
	}

	ret = 0;
out:
	EVP_PKEY_CTX_free(pctx);
	return ret;
}

static int do_decapsulate(EVP_PKEY *priv, const unsigned char *ct,
			  size_t ct_len, unsigned char **secret,
			  size_t *secret_len)
{
	EVP_PKEY_CTX *pctx = NULL;
	int ret = -1;

	pctx = EVP_PKEY_CTX_new_from_pkey(NULL, priv, "provider=pkcs11");
	if (!pctx) {
		fprintf(stderr, "EVP_PKEY_CTX_new_from_pkey (decaps) failed\n");
		print_openssl_errors();
		return -1;
	}

	if (EVP_PKEY_decapsulate_init(pctx, NULL) <= 0) {
		fprintf(stderr, "EVP_PKEY_decapsulate_init failed\n");
		print_openssl_errors();
		goto out;
	}

	*secret_len = 0;
	if (EVP_PKEY_decapsulate(pctx, NULL, secret_len, ct, ct_len) <= 0) {
		fprintf(stderr, "EVP_PKEY_decapsulate (size query) failed\n");
		print_openssl_errors();
		goto out;
	}

	*secret = malloc(*secret_len);
	if (!*secret) {
		fprintf(stderr, "malloc failed\n");
		goto out;
	}

	if (EVP_PKEY_decapsulate(pctx, *secret, secret_len, ct, ct_len) <= 0) {
		fprintf(stderr, "EVP_PKEY_decapsulate failed\n");
		print_openssl_errors();
		goto out;
	}

	ret = 0;
out:
	EVP_PKEY_CTX_free(pctx);
	return ret;
}

static void hex_dump(const char *label, const unsigned char *buf, size_t len)
{
	size_t i;

	printf("%s (%zu bytes): ", label, len);
	for (i = 0; i < len && i < 16; i++)
		printf("%02x", buf[i]);
	if (len > 16)
		printf("...");
	printf("\n");
}

int main(int argc, char *argv[])
{
	char pub_uri[512];
	char priv_uri[512];
	EVP_PKEY *pub = NULL;
	EVP_PKEY *priv = NULL;
	unsigned char *ct = NULL;
	unsigned char *secret_enc = NULL;
	unsigned char *secret_dec = NULL;
	size_t ct_len = 0;
	size_t secret_enc_len = 0;
	size_t secret_dec_len = 0;
	int rc = 1;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <object-label>\n", argv[0]);
		return 2;
	}

	snprintf(pub_uri, sizeof(pub_uri), "pkcs11:type=public;object=%s",
		 argv[1]);
	snprintf(priv_uri, sizeof(priv_uri), "pkcs11:type=private;object=%s",
		 argv[1]);

	printf("Loading public key: %s\n", pub_uri);
	pub = load_pkey(pub_uri);
	if (!pub)
		goto out;

	printf("Encapsulating...\n");
	if (do_encapsulate(pub, &ct, &ct_len, &secret_enc, &secret_enc_len))
		goto out;
	hex_dump("Ciphertext", ct, ct_len);
	hex_dump("Shared secret (encapsulate)", secret_enc, secret_enc_len);

	printf("Loading private key: %s\n", priv_uri);
	priv = load_pkey(priv_uri);
	if (!priv)
		goto out;

	printf("Decapsulating...\n");
	if (do_decapsulate(priv, ct, ct_len, &secret_dec, &secret_dec_len))
		goto out;
	hex_dump("Shared secret (decapsulate)", secret_dec, secret_dec_len);

	if (secret_enc_len != secret_dec_len ||
	    memcmp(secret_enc, secret_dec, secret_enc_len) != 0) {
		printf("FAIL: shared secrets do NOT match\n");
		goto out;
	}

	printf("PASS: shared secrets match (%zu bytes)\n", secret_enc_len);
	rc = 0;

out:
	free(ct);
	free(secret_enc);
	free(secret_dec);
	EVP_PKEY_free(pub);
	EVP_PKEY_free(priv);
	return rc;
}
